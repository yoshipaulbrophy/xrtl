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

#include "xrtl/port/linux/ui/x11_control.h"

#include "xrtl/base/debugging.h"
#include "xrtl/base/flags.h"
#include "xrtl/base/logging.h"

DEFINE_string(display, ":0.0",
              "X11 display to use, otherwise the DISPLAY envvar is used");

namespace xrtl {
namespace ui {

ref_ptr<Control> Control::Create(ref_ptr<MessageLoop> message_loop) {
  return make_ref<X11Control>(message_loop, nullptr);
}

X11Control::X11Control(ref_ptr<MessageLoop> message_loop,
                       ControlContainer* container)
    : Control(message_loop), container_(container) {
  create_event_ = Event::CreateManualResetEvent(false);
  destroy_event_ = Event::CreateManualResetEvent(false);
}

X11Control::~X11Control() { DCHECK(!window_handle_); }

::Display* X11Control::display_handle() const {
  DCHECK(display_);
  return display_->display_handle();
}

::Window X11Control::window_handle() const {
  DCHECK(window_handle_);
  return window_handle_;
}

Control::PlatformHandle X11Control::platform_handle() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  switch (state_) {
    case State::kCreating:
    case State::kDestroyed:
      return 0;
    default:
    case State::kCreated:
    case State::kDestroying:
      return window_handle_;
  }
}

Control::State X11Control::state() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return state_;
}

bool X11Control::is_active() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return state_ == State::kCreated;
}

bool X11Control::is_suspended() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return is_suspended_;
}

void X11Control::set_suspended(bool suspended) {
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
  if (display_ && window_handle_) {
    if (suspended) {
      ::XIconifyWindow(display_handle(), window_handle(),
                       XDefaultScreen(display_handle()));
    } else {
      // XMapWindow should be enough according to the docs, but doesn't work.
      // Can't hurt to call it in case some WMs expect it.
      ::XMapWindow(display_handle(), window_handle());

      // Post an event to switch to activate the window.
      // This only happens if the WM supports it (by having the
      // _NET_ACTIVE_WINDOW atom).
      if (net_active_window_atom_) {
        XClientMessageEvent ev;
        std::memset(&ev, 0, sizeof ev);
        ev.type = ClientMessage;
        ev.window = window_handle();
        ev.message_type = net_active_window_atom_;
        ev.format = 32;
        ev.data.l[0] = NormalState;  // 1
        ev.data.l[1] = CurrentTime;
        ev.data.l[2] = ev.data.l[3] = ev.data.l[4] = 0;
        ::XSendEvent(
            display_handle(),
            RootWindow(display_handle(), XDefaultScreen(display_handle())),
            False, SubstructureRedirectMask | SubstructureNotifyMask,
            reinterpret_cast<XEvent*>(&ev));
      }
    }
    ::XFlush(display_handle());
  }
}

bool X11Control::is_focused() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return !is_suspended_ && is_focused_;
}

void X11Control::set_focused(bool focused) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  is_focused_ = focused;
  if (display_ && window_handle_) {
    // X will generate errors if you try to set focus while the window is not
    // visible.
    XWindowAttributes attributes;
    ::XGetWindowAttributes(display_handle(), window_handle(), &attributes);
    if (attributes.map_state != IsViewable) {
      return;
    }
    if (focused) {
      ::XSetInputFocus(display_handle(), window_handle(), RevertToPointerRoot,
                       CurrentTime);
    } else {
      ::XSetInputFocus(display_handle(), PointerRoot, RevertToPointerRoot,
                       CurrentTime);
    }
    ::XFlush(display_handle());
  }
}

Rect2D X11Control::bounds() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bounds_;
}

void X11Control::set_bounds(Rect2D bounds) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bounds_ = bounds;
  if (display_ && window_handle_) {
    Frame2D frame = QueryFrame();
    bounds.origin.x -= frame.left;
    bounds.origin.y -= frame.top;
    bounds.size.width -= frame.right;
    bounds.size.height -= frame.bottom;

    ::XMoveResizeWindow(display_handle(), window_handle(), bounds.origin.x,
                        bounds.origin.y, bounds.size.width, bounds.size.height);
    ::XFlush(display_handle());
  }
}

gfx::rgba8_t X11Control::background_color() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return background_color_;
}

void X11Control::set_background_color(gfx::rgba8_t background_color) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  background_color_ = background_color;
  if (display_ && window_handle_) {
    XColor color;
    color.red = static_cast<int>((background_color.r / 255.0f) * 65535.0f);
    color.green = static_cast<int>((background_color.g / 255.0f) * 65535.0f);
    color.blue = static_cast<int>((background_color.b / 255.0f) * 65535.0f);
    ::XAllocColor(display_handle(), DefaultColormap(display_handle(), 0),
                  &color);
    ::XSetWindowBackground(display_handle(), window_handle(), color.pixel);
    InvalidateWithLock();
  }
}

bool X11Control::is_cursor_visible() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return is_cursor_visible_;
}

void X11Control::set_cursor_visible(bool cursor_visible) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (cursor_visible == is_cursor_visible_) {
    return;
  }
  is_cursor_visible_ = cursor_visible;
  if (display_ && window_handle_) {
    if (cursor_visible) {
      // Becoming visible. This must match a previous XDefineCursor (from our
      // invisible path below).
      ::XUndefineCursor(display_handle(), window_handle());
    } else {
      // NOTE: xlib actually freaking leaks here.
      debugging::LeakCheckDisabler leak_check_disabler;

      // Becoming invisible. We must XUndefineCursor this to remove it.
      static char kZeros[] = {0};
      ::Pixmap pixmap = ::XCreateBitmapFromData(display_handle(),
                                                window_handle(), kZeros, 1, 1);
      ::XColor black = {0};
      ::Cursor cursor = ::XCreatePixmapCursor(display_handle(), pixmap, pixmap,
                                              &black, &black, 0, 0);
      ::XDefineCursor(display_handle(), window_handle(), cursor);
      ::XFreeCursor(display_handle(), cursor);
      ::XFreePixmap(display_handle(), pixmap);
    }
    ::XFlush(display_handle());
  }
}

ref_ptr<WaitHandle> X11Control::Create() {
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

  // Begin opening the window. This is an async process with our initial X
  // requests happening here but all the rest happening over the course of
  // several XEvent callbacks.
  if (!BeginCreate()) {
    LOG(ERROR) << "Unable to begin creating control";
    PostError();
    message_loop_->Defer(&pending_task_list_,
                         [this]() { create_event_->Set(); });
    return create_event_;
  }

  // This event will be set when the window has finished opening.
  return create_event_;
}

bool X11Control::BeginCreate() {
  PostCreating();

  // Acquire display connection.
  // We must retain this for the life of the window.
  display_ =
      X11Display::Connect(FLAGS_display, message_loop_.As<EpollMessageLoop>());
  if (!display_) {
    LOG(ERROR) << "Unable to connect to X server";
    return false;
  }

  // Query a compatible visual for our target bit depth.
  ::XVisualInfo visual_info;
  ::XMatchVisualInfo(display_handle(), DefaultScreen(display_handle()), 24,
                     TrueColor, &visual_info);

  // Setup window attributes.
  ::XSetWindowAttributes window_attribs;
  std::memset(&window_attribs, 0, sizeof(window_attribs));
  window_attribs.colormap =
      ::XCreateColormap(display_handle(), DefaultRootWindow(display_handle()),
                        visual_info.visual, AllocNone);
  // Table of masks to events here:
  // https://tronche.com/gui/x/xlib/events/processing-overview.html
  window_attribs.event_mask =
      StructureNotifyMask | SubstructureNotifyMask | ExposureMask |
      EnterWindowMask | LeaveWindowMask | KeymapStateMask |
      VisibilityChangeMask | FocusChangeMask | PropertyChangeMask |
      ColormapChangeMask | PointerMotionMask | ButtonPressMask |
      ButtonReleaseMask | KeyPressMask | KeyReleaseMask;

  Rect2D initial_bounds = bounds();

  // We'll get at least two configure events on startup. This helps us wait for
  // the one we want.
  configure_count_ = 0;

  // Create the window in the display.
  window_handle_ = ::XCreateWindow(
      display_handle(), DefaultRootWindow(display_handle()),
      initial_bounds.origin.x, initial_bounds.origin.y,
      initial_bounds.size.width, initial_bounds.size.height, 0,
      visual_info.depth, InputOutput, visual_info.visual,
      CWBorderPixel | CWColormap | CWEventMask, &window_attribs);
  if (!window_handle_) {
    LOG(ERROR) << "Could not create window";
    return false;
  }
  VLOG(1) << "Created X window: " << std::hex << window_handle_;

  // Sync to ensure window has been created.
  ::XSync(display_handle(), False);

  // Add an event listener filtered to the window.
  display_->AddWindowListener(this, window_handle_);

  // Fake a configure event to force setup even if no window manager is present.
  // TODO(benvanik): verify this is needed. It introduces another bogus resize
  // event in the queue which breaks our tests.
  if (false) {
    ::XEvent config_event;
    config_event.xconfigure.type = ConfigureNotify;
    config_event.xconfigure.display = display_handle();
    config_event.xconfigure.window = window_handle();
    config_event.xconfigure.x = initial_bounds.origin.x;
    config_event.xconfigure.y = initial_bounds.origin.y;
    config_event.xconfigure.width = initial_bounds.size.width;
    config_event.xconfigure.height = initial_bounds.size.height;
    ::XPutBackEvent(display_handle(), &config_event);
  }

  // Hinting. Not required to succeed.
  ::XWMHints hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.input = True;
  hints.flags = InputHint;
  ::XSetWMHints(display_handle(), window_handle(), &hints);

  // Before we map the window, set size hints. Otherwise some window managers
  // will ignore top-level XMoveWindow commands.
  ::XSizeHints size_hints;
  std::memset(&size_hints, 0, sizeof(size_hints));
  if (initial_bounds.origin.x || initial_bounds.origin.y) {
    size_hints.flags |= PPosition;
    size_hints.x = initial_bounds.origin.x;
    size_hints.y = initial_bounds.origin.y;
  }
  // Set StaticGravity so that the window position is not affected by the
  // frame width when running with window manager.
  size_hints.flags |= PWinGravity;
  size_hints.win_gravity = StaticGravity;
  ::XSetWMNormalHints(display_handle(), window_handle(), &size_hints);

  // Intern atoms we use to prevent interning while running.
  wm_delete_window_atom_ =
      ::XInternAtom(display_handle(), "WM_DELETE_WINDOW", False);
  wm_state_atom_ = ::XInternAtom(display_handle(), "WM_STATE", False);
  net_active_window_atom_ =
      ::XInternAtom(display_handle(), "_NET_ACTIVE_WINDOW", True);
  net_frame_extents_atom_ =
      ::XInternAtom(display_handle(), "_NET_FRAME_EXTENTS", True);

  // Setup an atom to listen for window deletion events.
  ::XSetWMProtocols(display_handle(), window_handle(), &wm_delete_window_atom_,
                    1);

  // Map window to show it.
  ::XMapWindow(display_handle(), window_handle());
  ::XSync(display_handle(), False);

  // Set initial state.
  is_suspended_ = false;
  set_background_color(background_color());
  set_cursor_visible(is_cursor_visible());

  return true;
}

bool X11Control::EndCreate() {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = State::kCreated;

    // Signal any waiters.
    create_event_->Set();
  }

  if (container_) {
    container_->OnChildCreated(ref_ptr<X11Control>(this));
  }

  PostCreated();

  return true;
}

ref_ptr<WaitHandle> X11Control::Destroy() {
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

  // Begin closing the window. This is an async process with our initial X
  // requests happening here but all the rest happening over the course of
  // several XEvent callbacks.
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

bool X11Control::BeginDestroy() {
  PostDestroying();

  // Note that we let the DestroyNotify message handle the OnClose().
  ::XDestroyWindow(display_handle(), window_handle());
  ::XFlush(display_handle());

  return true;
}

bool X11Control::EndDestroy() {
  // Unregister the event handler.
  if (display_) {
    display_->RemoveWindowListener(this);
  }

  // Drop our X11 display connection.
  // After this we *really* can't make any more calls.
  display_.reset();

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = State::kDestroyed;
    window_handle_ = 0;

    // Signal any waiters.
    destroy_event_->Set();
  }

  if (container_) {
    container_->OnChildDestroyed(ref_ptr<X11Control>(this));
  }

  PostDestroyed();

  return true;
}

void X11Control::Invalidate() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!display_ || !window_handle_) {
    return;
  }
  InvalidateWithLock();
}

void X11Control::InvalidateWithLock() {
  ::XEvent ev;
  std::memset(&ev, 0, sizeof(ev));
  ev.type = Expose;
  ev.xexpose.window = window_handle();
  ::XSendEvent(display_handle(), window_handle(), False, ExposureMask, &ev);
  ::XFlush(display_handle());
}

Frame2D X11Control::QueryFrame() {
  if (!net_frame_extents_atom_) {
    return {};
  }

  Atom type = 0;
  int format;
  uint64_t item_count;
  uint64_t after;
  int32_t* data = nullptr;
  if (::XGetWindowProperty(display_handle(), window_handle(),
                           net_frame_extents_atom_, 0, 4, False,
                           AnyPropertyType, &type, &format, &item_count, &after,
                           reinterpret_cast<uint8_t**>(&data)) != Success ||
      item_count != 4 || after != 0) {
    LOG(WARNING) << "Unable to fetch window extents";
    return {};
  }

  Frame2D frame;
  frame.left = data[0];
  frame.top = data[2];
  frame.right = data[1];
  frame.bottom = data[3];

  if (data) {
    ::XFree(data);
  }

  return frame;
}

Point2D X11Control::QueryOrigin() {
  ::Window root_window;
  int x;
  int y;
  uint32_t width;
  uint32_t height;
  uint32_t border_width;
  uint32_t depth;
  ::XGetGeometry(display_handle(), window_handle(), &root_window, &x, &y,
                 &width, &height, &border_width, &depth);

  int offset_x = 0;
  int offset_y = 0;
  ::Window child_window;
  ::XTranslateCoordinates(display_handle(), window_handle(), root_window, 0, 0,
                          &offset_x, &offset_y, &child_window);

  return {offset_x, offset_y};
}

X11Control::WindowState X11Control::QueryWindowState() {
  WindowState result = WindowState::kNormal;
  Atom type = 0;
  int format;
  uint64_t item_count;
  uint64_t after;
  uint32_t* data = nullptr;
  ::XGetWindowProperty(display_handle(), window_handle(), wm_state_atom_, 0, 2,
                       False, AnyPropertyType, &type, &format, &item_count,
                       &after, reinterpret_cast<uint8_t**>(&data));
  if (type) {
    result = static_cast<WindowState>(*data);
  }
  if (data) {
    ::XFree(data);
  }
  return result;
}

bool X11Control::OnXEvent(::XEvent* x_event) {
  switch (x_event->type) {
    case KeyPress: {
      // Emitted when a key is pressed while the control has focus.
      // const auto& ev = x_event->xkey;
      VLOG(2) << "KeyPress";
      return true;
    }
    case KeyRelease: {
      // Emitted when a key is released while the control has focus.
      // const auto& ev = x_event->xkey;
      VLOG(2) << "KeyRelease";
      return true;
    }

    case ButtonPress: {
      // Emitted when a mouse button is pressed.
      // const auto& ev = x_event->xbutton;
      VLOG(2) << "ButtonPress";
      return true;
    }
    case ButtonRelease: {
      // Emitted when a mouse button is released.
      // const auto& ev = x_event->xbutton;
      VLOG(2) << "ButtonRelease";
      return true;
    }

    case MotionNotify: {
      // Emitted on pointer motion.
      const auto& ev = x_event->xmotion;
      VLOG(2) << "MotionNotify " << ev.x << "," << ev.y;
      return true;
    }

    case EnterNotify: {
      // Emitted when the mouse cursor enters the control.
      // const auto& ev = x_event->xcrossing;
      VLOG(1) << "EnterNotify";
      return true;
    }
    case LeaveNotify: {
      // Emitted when the mouse cursor leaves the control.
      // const auto& ev = x_event->xcrossing;
      VLOG(1) << "LeaveNotify";
      return true;
    }

    case FocusIn: {
      // Emitted when the control gains focus.
      // const auto& ev = x_event->xfocus;
      VLOG(1) << "FocusIn";
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      is_focused_ = true;
      if (state_ == State::kCreated) {
        if (!is_suspended_) {
          PostFocusChanged(true);
        }
      }
      return true;
    }
    case FocusOut: {
      // Emitted when the control loses focus.
      // const auto& ev = x_event->xfocus;
      VLOG(1) << "FocusOut";
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      is_focused_ = false;
      if (state_ == State::kCreated) {
        PostFocusChanged(false);
      }
      return true;
    }

    case KeymapNotify: {
      // Emitted when the system keymap changes.
      // const auto& ev = x_event->xkeymap;
      VLOG(1) << "KeymapNotify";
      return true;
    }

    case Expose: {
      // Emitted when the control must be redrawn.
      // If ev.count > 0 there are more pending expose events. We should only
      // repaint when the last event has been emitted.
      // This is designed so that we can accumulate dirty rects, but we draw
      // everything anyway so we don't bother.
      const auto& ev = x_event->xexpose;
      VLOG(1) << "Expose with pending: " << ev.count;
      return true;
    }

    case VisibilityNotify: {
      // Emitted when window visibility changes.
      const auto& ev = x_event->xvisibility;
      VLOG(1) << "VisibilityNotify " << (ev.state != VisibilityFullyObscured);
      return true;
    }

    case CreateNotify: {
      // TODO(benvanik): figure out when we get this - possibly subwindows?
      // const auto& ev = x_event->xcreatewindow;
      VLOG(1) << "CreateNotify";
      return true;
    }
    case DestroyNotify: {
      // Emitted when a window has been destroyed via XDestroyWindow.
      // const auto& ev = x_event->xdestroywindow;
      VLOG(1) << "DestroyNotify";
      // We need to defer this as there are still some events pending.
      message_loop_->Defer(&pending_task_list_, [this]() { EndDestroy(); });
      return true;
    }

    case UnmapNotify: {
      // Emitted when the window is unmapped.
      // const auto& ev = x_event->xunmap;
      VLOG(1) << "UnmapNotify";
      return true;
    }

    case MapNotify: {
      // Emitted when the control has been mapped (materialized, created, etc).
      // const auto& ev = x_event->xmap;
      VLOG(1) << "MapNotify";
      return true;
    }

    case ReparentNotify: {
      // Emitted when the control has successfully reparented.
      const auto& ev = x_event->xreparent;
      VLOG(1) << "ReparentNotify into " << std::hex << ev.parent;
      return true;
    }

    case ConfigureNotify: {
      // Emitted whenever the window configuration changes, such as resizes.
      // NOTE: xy are bogus in the event.
      const auto& ev = x_event->xconfigure;
      VLOG(1) << "ConfigureNotify " << ev.x << "," << ev.y << " " << ev.width
              << "x" << ev.height;

      std::lock_guard<std::recursive_mutex> lock(mutex_);
      bool is_creating = state_ == State::kCreating;
      bool is_destroying = state_ == State::kDestroying;
      if (is_destroying) {
        DCHECK(!is_creating);
        return true;
      }

      // Get *actual* origin/size, as the event contains a bogus origin.
      // We only try to do this while the window is valid.
      Rect2D bounds;
      bounds.origin = QueryOrigin();
      bounds.size = {ev.width, ev.height};

      if (!is_creating) {
        Frame2D frame = QueryFrame();
        bounds.origin.x += frame.left;
        bounds.origin.y += frame.top;
        bounds.size.width += frame.right;
        bounds.size.height += frame.bottom;
      }
      bounds_ = bounds;

      // ConfigureNotify will be one of the first events sent, use this to
      // bracket our create flow.
      // X will send 2 - one immediately after the create window and another
      // after it is properly placed on the screen. To avoid extraneous resizing
      // we wait until the second event.
      // TODO(benvanik): ensure it's always 2, or find a way to differentiate.
      if (is_creating) {
        ++configure_count_;
        if (configure_count_ == 2) {
          configure_count_ = 0;
          EndCreate();
        } else {
          return true;
        }
      }

      if (state_ == State::kCreated) {
        PostSuspendChanged(is_suspended_);
        if (!is_suspended_) {
          PostResized(bounds_);
        }
        if (is_suspended_) {
          is_focused_ = false;
        }
        PostFocusChanged(is_focused_);
      }
      return true;
    }

    case GravityNotify: {
      // Emitted when the window changes position based on a parent resize.
      // NOTE: xy are bogus in the event.
      const auto& ev = x_event->xgravity;
      VLOG(1) << "GravityNotify " << ev.x << "," << ev.y;
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      Rect2D bounds;
      bounds.origin = QueryOrigin();
      bounds.size = bounds_.size;
      bounds_ = bounds;
      if (state_ == State::kCreated) {
        PostResized(bounds_);
      }
      return true;
    }

    case PropertyNotify: {
      // Emitted when a window property is created/updated or deleted.
      const auto& ev = x_event->xproperty;
      VLOG(1) << "PropertyNotify " << ev.atom << " " << ev.state;
      if (ev.atom == wm_state_atom_ && ev.state == PropertyNewValue) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        WindowState new_state = QueryWindowState();
        switch (new_state) {
          case WindowState::kWithdrawn:
            break;
          case WindowState::kNormal:
            is_suspended_ = false;
            break;
          case WindowState::kIconic:
            is_suspended_ = true;
            is_focused_ = false;
            break;
        }
        if (state_ == State::kCreated) {
          PostSuspendChanged(is_suspended_);
          PostFocusChanged(is_focused_);
        }
      }
      return true;
    }

    case ColormapNotify: {
      // Emitted when the colormap changes. This will happen once on control
      // creation and possibly multiple times after that.
      // const auto& ev = x_event->xcolormap;
      VLOG(1) << "ColormapNotify";
      return true;
    }

    case ClientMessage: {
      // Emitted when the control receives a custom message.
      const auto& ev = x_event->xclient;
      VLOG(1) << "ClientMessage " << std::hex << ev.data.l[0];
      if (static_cast<Atom>(ev.data.l[0]) ==
          static_cast<Atom>(wm_delete_window_atom_)) {
        Destroy();
      }
      return true;
    }

    case MappingNotify: {
      // Emitted when a device mapping changes.
      auto& ev = x_event->xmapping;
      VLOG(1) << "MappingNotify " << ev.request;
      switch (ev.request) {
        case MappingModifier:
        case MappingKeyboard:
          ::XRefreshKeyboardMapping(&ev);
          break;
        case MappingPointer:
          break;  // ?
      }
      return true;
    }

    default: {
      VLOG(1) << "Unhandled XEvent " << x_event->type;
      break;
    }
  }
  return false;
}

}  // namespace ui
}  // namespace xrtl
