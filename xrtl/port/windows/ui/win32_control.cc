// Copyright 2017 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "xrtl/port/windows/ui/win32_control.h"

#include "xrtl/base/logging.h"

// TODO(benvanik): figure out why uxtheme.h chokes.
#define MILCORE_KERNEL_COMPONENT
typedef struct _MARGINS {
  int cxLeftWidth;     // width of left border that retains its size
  int cxRightWidth;    // width of right border that retains its size
  int cyTopHeight;     // height of top border that retains its size
  int cyBottomHeight;  // height of bottom border that retains its size
} MARGINS, *PMARGINS;

#include <dwmapi.h>   // DWM MMCSS/etc.
#include <tpcshrd.h>  // Tablet defines.

namespace xrtl {
namespace ui {

namespace {

const wchar_t* kWindowClassName = L"XrtlControlClass";

const DWORD_PTR kTabletGestureStatus =
    // Press and hold (right-click) gesture.
    TABLET_DISABLE_PRESSANDHOLD |
    // UI feedback on pen up (waves).
    TABLET_DISABLE_PENTAPFEEDBACK |
    // UI feedback on pen button down (circle).
    TABLET_DISABLE_PENBARRELFEEDBACK |
    // Pen flicks (back, forward, drag down, drag up).
    TABLET_DISABLE_FLICKS |
    // Misc.
    TABLET_DISABLE_TOUCHSWITCH | TABLET_DISABLE_SMOOTHSCROLLING |
    TABLET_DISABLE_TOUCHUIFORCEON | TABLET_ENABLE_MULTITOUCHDATA;

// Number of calls to enable DWM MMCSS outstanding.
std::atomic<int> dwm_mmcss_counter_{0};

typedef HRESULT(__stdcall* LPFNDwmEnableMMCSS)(BOOL);
LPFNDwmEnableMMCSS pDwmEnableMMCSS = nullptr;
typedef HRESULT(__stdcall* LPFNDwmSetPresentParameters)(
    HWND, DWM_PRESENT_PARAMETERS*);
LPFNDwmSetPresentParameters pDwmSetPresentParameters = nullptr;

// Ensures the DWM symbols have been loaded.
bool PopulateDwmSymbols() {
  static std::once_flag dwm_symbols_flag;
  std::call_once(dwm_symbols_flag, []() {
    // Try to load DWM. This should be available everywhere, but may not be in
    // certain environments (like wine).
    // NOTE: we explicitly don't ::FreeLibrary the handle, as we want to keep
    // the function pointers valid for the lifetime of the process.
    HMODULE dwmapi_library = ::LoadLibraryA("DWMAPI.DLL");
    if (!dwmapi_library) {
      return;
    }
    pDwmEnableMMCSS = reinterpret_cast<LPFNDwmEnableMMCSS>(
        ::GetProcAddress(dwmapi_library, "DwmEnableMMCSS"));
    pDwmSetPresentParameters = reinterpret_cast<LPFNDwmSetPresentParameters>(
        ::GetProcAddress(dwmapi_library, "DwmSetPresentParameters"));
  });
  return pDwmEnableMMCSS != nullptr;
}

// Enables DWM MMCSS (Desktop Window Manager Multimedia Class Schedule Service).
// This is a system-wide flag that makes the compositor (DWM) update with much
// lower latency. If we don't enable it while trying to render it's possible for
// the compositor to run at much less than refresh rate.
void EnableMediaPresentation(HWND hwnd) {
  if (!PopulateDwmSymbols()) {
    return;
  }
  if (dwm_mmcss_counter_.fetch_add(1) == 0) {
    // Enabling and we are the first user.
    pDwmEnableMMCSS(TRUE);
  }

  DWM_PRESENT_PARAMETERS pp = {0};
  pp.cbSize = sizeof(DWM_PRESENT_PARAMETERS);
  pp.fQueue = FALSE;
  pp.cBuffer = 2;
  pp.fUseSourceRate = FALSE;
  pp.cRefreshesPerFrame = 1;
  pp.eSampling = DWM_SOURCE_FRAME_SAMPLING_POINT;
  pDwmSetPresentParameters(hwnd, &pp);
}

// Disables DWM MMCSS.
void DisableMediaPresentation() {
  if (!PopulateDwmSymbols()) {
    return;
  }
  if (dwm_mmcss_counter_.fetch_sub(1) == 1) {
    // Disabling and we now have no more users.
    pDwmEnableMMCSS(FALSE);
  }
}

}  // namespace

ref_ptr<Control> Control::Create(ref_ptr<MessageLoop> message_loop) {
  return make_ref<Win32Control>(message_loop, nullptr);
}

Win32Control::Win32Control(ref_ptr<MessageLoop> message_loop,
                           ControlContainer* container)
    : Control(message_loop), container_(container) {
  create_event_ = Event::CreateManualResetEvent(false);
  destroy_event_ = Event::CreateManualResetEvent(false);
}

Win32Control::~Win32Control() { DCHECK(!hwnd_); }

HWND Win32Control::hwnd() const {
  DCHECK(hwnd_);
  return hwnd_;
}

Control::PlatformHandle Win32Control::platform_handle() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  switch (state_) {
    case State::kCreating:
    case State::kDestroyed:
      return 0;
    case State::kCreated:
    case State::kDestroying:
      return reinterpret_cast<Control::PlatformHandle>(hwnd_);
  }
}

Control::State Win32Control::state() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return state_;
}

bool Win32Control::is_active() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return state_ == State::kCreated;
}

bool Win32Control::is_suspended() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return is_suspended_;
}

void Win32Control::set_suspended(bool suspended) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  is_suspended_ = suspended;
  switch (state_) {
    case State::kCreating:
    case State::kCreated:
      break;
    case State::kDestroying:
    case State::kDestroyed:
      return;  // Ignored.
  }
  if (hwnd_) {
    if (suspended) {
      ::PostMessageW(hwnd_, WM_SYSCOMMAND, SC_MINIMIZE, 0);
    } else {
      ::PostMessageW(hwnd_, WM_SYSCOMMAND, SC_RESTORE, 0);
    }
  }
}

bool Win32Control::is_focused() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return !is_suspended_ && is_focused_;
}

void Win32Control::set_focused(bool focused) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  is_focused_ = focused;
  if (hwnd_) {
    message_loop_->Defer(&pending_task_list_, [this, focused]() {
      if (focused) {
        ::SetFocus(hwnd());
      } else {
        ::SetFocus(nullptr);
      }
    });
  }
}

Rect2D Win32Control::bounds() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bounds_;
}

void Win32Control::set_bounds(Rect2D bounds) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bounds_ = bounds;
  if (hwnd_) {
    RECT rc = {bounds.origin.x, bounds.origin.y,
               bounds.origin.x + bounds.size.width,
               bounds.origin.y + bounds.size.height};
    bool has_menu = false;  // TODO(benvanik): main menu logic.
    ::AdjustWindowRect(&rc, ::GetWindowLong(hwnd(), GWL_STYLE), has_menu);
    message_loop_->Defer(&pending_task_list_, [this, rc]() {
      ::MoveWindow(hwnd(), rc.left, rc.top, rc.right - rc.left,
                   rc.bottom - rc.top, TRUE);
    });
  }
}

gfx::rgba8_t Win32Control::background_color() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return background_color_;
}

void Win32Control::set_background_color(gfx::rgba8_t background_color) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  background_color_ = background_color;
  if (hwnd_) {
    ::InvalidateRect(hwnd(), nullptr, FALSE);
  }
}

bool Win32Control::is_cursor_visible() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return is_cursor_visible_;
}

void Win32Control::set_cursor_visible(bool cursor_visible) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (cursor_visible == is_cursor_visible_) {
    return;
  }
  is_cursor_visible_ = cursor_visible;
  if (hwnd_) {
    if (cursor_visible) {
      // Becoming visible.
      ::ShowCursor(TRUE);
      ::SetCursor(nullptr);
    } else {
      // Becoming invisible.
      ::ShowCursor(FALSE);
    }
  }
}

ref_ptr<WaitHandle> Win32Control::Create() {
  // Handle this being called if the window is already open or closing.
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    switch (state_) {
      case State::kCreating:
        // Window is currently opening. Return the wait handle so the caller
        // can be notified of completion.
        return create_event_;
      case State::kCreated:
        // Window is already open. No-op.
        return create_event_;
      case State::kDestroying:
        // Window is currently closing. Avoid races like this, please.
        LOG(ERROR) << "Unable to create control while it is being destroyed";
        PostError();
        message_loop_->Defer(&pending_task_list_,
                             [this]() { create_event_->Set(); });
        DCHECK(false);
        return create_event_;
      case State::kDestroyed:
        // Window is closed. We can open it again.
        destroy_event_->Reset();
        state_ = State::kCreating;
        break;
    }
  }

  // Reset event shadows so that our listener receives all events at least once.
  ResetEventShadows();

  // Begin opening the window. This is an async process and *must* happen on
  // the message loop thread.
  message_loop_->Defer(&pending_task_list_, [this]() {
    if (!BeginCreate()) {
      LOG(ERROR) << "Unable to create control";
      PostError();
      message_loop_->Defer(&pending_task_list_,
                           [this]() { create_event_->Set(); });
    }
  });

  // This event will be set when the window has finished opening.
  return create_event_;
}

bool Win32Control::BeginCreate() {
  PostCreating();

  // Ensure we create the window class we use for the window.
  // This should be process-local so we only need to do it once.
  static std::once_flag register_class_flag;
  std::call_once(register_class_flag, []() {
    WNDCLASSEXW wcex = {0};
    wcex.cbSize = sizeof(WNDCLASSEXW);
    wcex.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wcex.lpfnWndProc = Win32Control::WndProcThunk;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = ::GetModuleHandle(nullptr);
    wcex.hIcon = nullptr;
    wcex.hIconSm = nullptr;
    wcex.hCursor = ::LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wcex.lpszMenuName = nullptr;
    wcex.lpszClassName = kWindowClassName;
    if (!::RegisterClassExW(&wcex)) {
      LOG(FATAL) << "Unable to register window class";
    }
  });

  // Setup initial window style.
  DWORD window_style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
  DWORD window_ex_style = WS_EX_APPWINDOW | WS_EX_CONTROLPARENT;

  // Determine starting position.
  Rect2D initial_bounds = bounds();
  RECT rc = {initial_bounds.origin.x, initial_bounds.origin.y,
             initial_bounds.origin.x + initial_bounds.size.width,
             initial_bounds.origin.y + initial_bounds.size.height};
  ::AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
  int initial_x = rc.left;
  int initial_y = rc.top;
  int initial_width = rc.right - rc.left;
  int initial_height = rc.bottom - rc.top;
  if (initial_bounds.origin.x == 0 && initial_bounds.origin.y == 0) {
    // TODO(benvanik): a cleaner way of setting position to default.
    initial_x = CW_USEDEFAULT;
    initial_y = CW_USEDEFAULT;
  }

  // Create window.
  hwnd_ = ::CreateWindowExW(window_ex_style, kWindowClassName, L"(xrtl)",
                            window_style, initial_x, initial_y, initial_width,
                            initial_height, nullptr, nullptr,
                            ::GetModuleHandle(nullptr), this);
  if (!hwnd_) {
    LOG(ERROR) << "Unable to create window";
    return false;
  }
  VLOG(1) << "Created Win32 window: " << std::hex << hwnd();

  // Disable flicks and other tablet gestures.
  // Some of these happen via the WM_TABLET_QUERYSYSTEMGESTURESTATUS message.
  ATOM tablet_service_atom =
      ::GlobalAddAtomW(L"MicrosoftTabletPenServiceProperty");
  ::SetProp(hwnd_, MAKEINTATOM(tablet_service_atom),
            reinterpret_cast<HANDLE>(kTabletGestureStatus));
  ::GlobalDeleteAtom(tablet_service_atom);

  // Enable DWM MMCSS.
  EnableMediaPresentation(hwnd());

  // Put the end creation event in the loop so some of our other messages can
  // go first.
  message_loop_->Defer(&pending_task_list_, [this]() {
    if (!EndCreate()) {
      LOG(ERROR) << "Unable to end creating control";
      PostError();
      message_loop_->Defer(&pending_task_list_,
                           [this]() { create_event_->Set(); });
    }
  });

  // Show, minimized if requested.
  if (is_suspended()) {
    ::ShowWindowAsync(hwnd(), SW_SHOWMINIMIZED);
  } else {
    ::ShowWindowAsync(hwnd(), is_focused() ? SW_SHOW : SW_SHOWNA);
  }

  // Set initial state.
  is_suspended_ = false;
  set_background_color(background_color());
  set_cursor_visible(is_cursor_visible());
  set_bounds(QueryBounds());

  return true;
}

bool Win32Control::EndCreate() {
  DCHECK(hwnd_);

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = State::kCreated;

    // Signal any waiters.
    create_event_->Set();
  }

  if (container_) {
    container_->OnChildCreated(ref_ptr<Win32Control>(this));
  }

  PostCreated();

  return true;
}

ref_ptr<WaitHandle> Win32Control::Destroy() {
  // Handle this being called if the window is already closing or closed.
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    switch (state_) {
      case State::kCreating:
        // Window is currently creating. Avoid races like this, please.
        LOG(ERROR) << "Unable to destroy control while it is being created";
        PostError();
        message_loop_->Defer(&pending_task_list_,
                             [this]() { destroy_event_->Set(); });
        DCHECK(false);
        return destroy_event_;
      case State::kCreated:
        // Window is open. We can close it.
        create_event_->Reset();
        state_ = State::kDestroying;
        break;
      case State::kDestroying:
        // Window is currently closing, so just return the wait handle for that
        // previous request.
        return destroy_event_;
      case State::kDestroyed:
        // Window is already closed. No-op. The event should be set.
        return destroy_event_;
    }
  }

  // Begin closing the window. This is an async process with our initial win32
  // requests happening here but all the rest happening over the course of
  // several window message callbacks.
  if (!BeginDestroy()) {
    LOG(ERROR) << "Unable to begin closing window";
    PostError();
    message_loop_->Defer(&pending_task_list_,
                         [this]() { destroy_event_->Set(); });
    return destroy_event_;
  }

  // This event will be set when the window has finished closing.
  return destroy_event_;
}

bool Win32Control::BeginDestroy() {
  PostDestroying();

  // We'll call EndDestroy from the close message handler.
  message_loop_->Defer(&pending_task_list_,
                       [this]() { ::DestroyWindow(hwnd()); });

  return true;
}

bool Win32Control::EndDestroy() {
  DisableMediaPresentation();

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = State::kDestroyed;
    if (hwnd_) {
      ::SetWindowLongPtr(hwnd(), GWLP_USERDATA, 0);
      hwnd_ = nullptr;
    }

    // Signal any waiters.
    destroy_event_->Set();
  }

  if (container_) {
    container_->OnChildDestroyed(ref_ptr<Win32Control>(this));
  }

  PostDestroyed();

  return true;
}

void Win32Control::Invalidate() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!hwnd_) {
    return;
  }
  ::InvalidateRect(hwnd(), nullptr, FALSE);
}

Rect2D Win32Control::QueryBounds() {
  WINDOWPLACEMENT placement = {0};
  placement.length = sizeof(WINDOWPLACEMENT);
  ::GetWindowPlacement(hwnd(), &placement);
  if (placement.showCmd == SW_SHOWMINIMIZED) {
    // Can't query bounds while minimized.
    return bounds_;
  }

  // TODO(benvanik): use GetClientRect for subcontrols (not screen pos).
  //   ::GetClientRect(hwnd(), &client_rect);
  RECT client_rect;
  ::GetWindowRect(hwnd(), &client_rect);

  bool has_menu = false;  // TODO(benvanik): main menu logic.
  RECT adjust_rect = {0};
  ::AdjustWindowRect(&adjust_rect, ::GetWindowLong(hwnd(), GWL_STYLE),
                     has_menu);

  Rect2D bounds;
  bounds.origin.x = client_rect.left - adjust_rect.left;
  bounds.origin.y = client_rect.top - adjust_rect.top;
  bounds.size.width = client_rect.right - client_rect.left -
                      (adjust_rect.right - adjust_rect.left);
  bounds.size.height = client_rect.bottom - client_rect.top -
                       (adjust_rect.bottom - adjust_rect.top);

  return bounds;
}

LRESULT CALLBACK Win32Control::WndProcThunk(HWND hwnd, UINT message,
                                            WPARAM w_param, LPARAM l_param) {
  // Retrieve the target control from the hwnd.
  Win32Control* control = nullptr;
  if (message == WM_NCCREATE) {
    // The window has been created with the system.
    // This is called *inline* in the ::CreateWindow call, so we have to be
    // very careful what state we access.
    auto create_struct = reinterpret_cast<LPCREATESTRUCT>(l_param);
    control = reinterpret_cast<Win32Control*>(create_struct->lpCreateParams);
    control->hwnd_ = hwnd;

    // Attach our pointer in user data so that we can get it back in the
    // message thunk.
    ::SetWindowLongPtr(
        hwnd, GWLP_USERDATA,
        static_cast<__int3264>(reinterpret_cast<LONG_PTR>(control)));
  } else {
    control = reinterpret_cast<Win32Control*>(
        ::GetWindowLongPtr(hwnd, GWLP_USERDATA));
  }
  if (control) {
    return control->WndProc(hwnd, message, w_param, l_param);
  } else {
    return ::DefWindowProc(hwnd, message, w_param, l_param);
  }
}

LRESULT Win32Control::WndProc(HWND hwnd, UINT message, WPARAM w_param,
                              LPARAM l_param) {
  DCHECK_EQ(hwnd_, hwnd);

  if (message >= WM_MOUSEFIRST && message <= WM_MOUSELAST) {
    // TODO(benvanik): mouse events.
    VLOG(2) << "(mouse message)";
    return ::DefWindowProc(hwnd, message, w_param, l_param);
  } else if (message >= WM_KEYFIRST && message <= WM_KEYLAST) {
    // TODO(benvanik): keyboard events.
    VLOG(2) << "(keyboard message)";
    return ::DefWindowProc(hwnd, message, w_param, l_param);
  }

  switch (message) {
    case WM_NCCREATE: {
      // Handled above in the WndProcThunk to signal creation completing.
      VLOG(1) << "WM_NCCREATE";
      break;
    }

    case WM_CREATE: {
      VLOG(1) << "WM_CREATE";
      break;
    }
    case WM_CLOSE: {
      VLOG(1) << "WM_CLOSE";
      Destroy();
      break;
    }
    case WM_DESTROY: {
      VLOG(1) << "WM_DESTROY";
      // We need to defer this as there are still some events pending.
      message_loop_->Defer(&pending_task_list_, [this]() { EndDestroy(); });
      break;
    }

    case WM_TABLET_QUERYSYSTEMGESTURESTATUS: {
      // Disable extended tablet features that mess with input.
      return kTabletGestureStatus;
    }

    case WM_MOVING: {
      VLOG(1) << "WM_MOVING";
      break;
    }
    case WM_MOVE: {
      VLOG(1) << "WM_MOVE";
      Rect2D bounds = QueryBounds();
      if (bounds.size.width == 0 || bounds.size.height == 0) {
        // Ignore when minimized.
        break;
      }
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      bounds_ = bounds;
      PostResized(bounds_);
      break;
    }

    case WM_SIZING: {
      VLOG(1) << "WM_SIZING";
      break;
    }
    case WM_SIZE: {
      VLOG(1) << "WM_SIZE";
      if (w_param == SIZE_MINIMIZED) {
        // Ignore when minimized.
        break;
      }
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      bounds_ = QueryBounds();
      PostResized(bounds_);
      break;
    }

    case WM_PAINT: {
      VLOG(1) << "WM_PAINT";
      // Mark entire window as valid.
      ::ValidateRect(hwnd, nullptr);
      return 0;  // Skip default because of custom paint.
    }
    case WM_ERASEBKGND: {
      return 0;  // Skip default because of custom paint.
    }

    case WM_DISPLAYCHANGE: {
      VLOG(1) << "WM_DISPLAYCHANGE";
      break;
    }

    case WM_SHOWWINDOW: {
      bool is_visible = w_param == TRUE;
      VLOG(1) << "WM_SHOWWINDOW " << is_visible;
      if (is_visible) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        is_suspended_ = false;
        PostSuspendChanged(is_suspended_);
        bounds_ = QueryBounds();
        PostResized(bounds_);
        PostFocusChanged(is_focused_);
      }
      break;
    }

    case WM_SYSCOMMAND: {
      switch (w_param & 0xFFF0) {
        case SC_MINIMIZE: {
          VLOG(1) << "WM_SYSCOMMAND: SC_MINIMIZE";
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          is_suspended_ = true;
          PostSuspendChanged(is_suspended_);
          is_focused_ = false;
          PostFocusChanged(is_focused_);
          break;
        }
        case SC_RESTORE: {
          VLOG(1) << "WM_SYSCOMMAND: SC_RESTORE";
          std::lock_guard<std::recursive_mutex> lock(mutex_);
          is_suspended_ = false;
          PostSuspendChanged(is_suspended_);
          bounds_ = QueryBounds();
          PostResized(bounds_);
          PostFocusChanged(is_focused_);
          break;
        }
      }
      break;
    }

    case WM_KILLFOCUS: {
      VLOG(1) << "WM_KILLFOCUS";
      if (reinterpret_cast<HWND>(w_param) == hwnd) {
        // Killing focus to then send it to ourselves... wat.
        break;
      }
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      is_focused_ = false;
      if (state_ == State::kCreated) {
        PostFocusChanged(is_focused_);
      }
      break;
    }
    case WM_SETFOCUS: {
      VLOG(1) << "WM_SETFOCUS";
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      is_focused_ = true;
      if (state_ == State::kCreated) {
        PostFocusChanged(is_focused_);
      }
      break;
    }
  }

  return ::DefWindowProc(hwnd, message, w_param, l_param);
}

}  // namespace ui
}  // namespace xrtl
