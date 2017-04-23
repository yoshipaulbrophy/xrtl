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

#include <math.h>

#include "xrtl/base/logging.h"
#include "xrtl/base/math.h"
#include "xrtl/base/system_clock.h"
#include "xrtl/base/threading/event.h"
#include "xrtl/base/threading/thread.h"
#include "xrtl/testing/demo_main.h"
#include "xrtl/ui/window.h"

namespace xrtl {
namespace ui {
namespace {

constexpr int PLAY_ORIGIN_X = 50;
constexpr int PLAY_ORIGIN_Y = 80;
constexpr int PLAY_WIDTH = 1600;
constexpr int PLAY_HEIGHT = 900;
constexpr int PADDLE_X_OFFSET = 80;
constexpr int PADDLE_HEIGHT = 200;
constexpr int PADDLE_WIDTH = 60;
constexpr int BALL_WIDTH = 75;
constexpr int BALL_HEIGHT = 75;

bool rects_intersect(const Rect2D& rect1, const Rect2D& rect2) {
  return !(rect2.origin.x > rect1.origin.x + rect1.size.width ||
           rect2.origin.x + rect2.size.width < rect1.origin.x ||
           rect2.origin.y > rect1.origin.y + rect1.size.height ||
           rect2.origin.y + rect2.size.height < rect1.origin.y);
}

// A pong game built using multiple moving windows.
class WindowPongDemo : private Control::Listener {
 public:
  WindowPongDemo() {
    message_loop_ = MessageLoop::Create();
    done_event_ = Event::CreateFence();
  }

  ~WindowPongDemo() { Thread::Wait(message_loop_->Exit()); }

  void Run() {
    ResetPositions();
    SetUpWindows();

    // Setup Tick() loop.
    message_loop_->DeferRepeating(&task_list_, [this]() { Tick(); },
                                  std::chrono::milliseconds(16));

    // Block until the game is exited.
    Thread::Wait(done_event_);

    CloseWindows();
  }

  void ResetPositions() {
    ball_x_position_ = PLAY_WIDTH / 2 - BALL_WIDTH / 2;
    ball_y_position_ = PLAY_HEIGHT / 2 - BALL_HEIGHT / 2;
    ball_rect_ = {static_cast<int>(ball_x_position_),
                  static_cast<int>(ball_y_position_), BALL_WIDTH, BALL_HEIGHT};
    // TODO(scotttodd): Randomize starting direction.
    ball_x_velocity_ = 0.707f;
    ball_y_velocity_ = 0.707f;

    left_paddle_rect_ = {PADDLE_X_OFFSET, PLAY_HEIGHT / 2 - PADDLE_HEIGHT / 2,
                         PADDLE_WIDTH, PADDLE_HEIGHT};
    right_paddle_rect_ = {PLAY_WIDTH - PADDLE_X_OFFSET - PADDLE_WIDTH,
                          PLAY_HEIGHT / 2 - PADDLE_HEIGHT / 2, PADDLE_WIDTH,
                          PADDLE_HEIGHT};
  }

  void SetUpWindows() {
    window_background_ = Window::Create(message_loop_);
    window_background_->set_title("Window Pong");
    window_background_->root_control()->set_listener(this);
    window_background_->root_control()->set_bounds(
        {PLAY_ORIGIN_X, PLAY_ORIGIN_Y, PLAY_WIDTH, PLAY_HEIGHT});
    window_background_->root_control()->set_background_color({20, 20, 20, 55});

    window_left_paddle_ = Window::Create(message_loop_);
    window_left_paddle_->set_title("Player 1");
    window_left_paddle_->root_control()->set_listener(this);
    window_left_paddle_->root_control()->set_background_color(
        {200, 100, 100, 255});
    MoveWindowLeftPaddle();

    window_right_paddle_ = Window::Create(message_loop_);
    window_right_paddle_->set_title("Player 2");
    window_right_paddle_->root_control()->set_listener(this);
    window_right_paddle_->root_control()->set_background_color(
        {100, 100, 200, 255});
    MoveWindowRightPaddle();

    window_ball_ = Window::Create(message_loop_);
    window_ball_->set_title("Ball");
    window_ball_->root_control()->set_listener(this);
    window_ball_->root_control()->set_background_color({100, 100, 100, 255});
    MoveWindowBall();

    Thread::WaitAll({window_background_->Open(), window_left_paddle_->Open(),
                     window_right_paddle_->Open(), window_ball_->Open()});
  }

  void CloseWindows() {
    Thread::WaitAll({window_left_paddle_->Close(),
                     window_right_paddle_->Close(), window_ball_->Close(),
                     window_background_->Close()});
  }

  void MoveWindowLeftPaddle() {
    window_left_paddle_->root_control()->set_bounds(
        {PLAY_ORIGIN_X + left_paddle_rect_.origin.x,
         PLAY_ORIGIN_Y + left_paddle_rect_.origin.y,
         left_paddle_rect_.size.width, left_paddle_rect_.size.height});
  }

  void MoveWindowRightPaddle() {
    window_right_paddle_->root_control()->set_bounds(
        {PLAY_ORIGIN_X + right_paddle_rect_.origin.x,
         PLAY_ORIGIN_Y + right_paddle_rect_.origin.y,
         right_paddle_rect_.size.width, right_paddle_rect_.size.height});
  }

  void MoveWindowBall() {
    window_ball_->root_control()->set_bounds(
        {PLAY_ORIGIN_X + ball_rect_.origin.x,
         PLAY_ORIGIN_Y + ball_rect_.origin.y, ball_rect_.size.width,
         ball_rect_.size.height});
  }

  void Tick() {
    double now = SystemClock::default_clock()->now_millis_highp();
    double dt = now - last_tick_time_;

    // Move the ball.
    ball_x_position_ += ball_speed_ * ball_x_velocity_ * dt / 1000.0;
    ball_y_position_ += ball_speed_ * ball_y_velocity_ * dt / 1000.0;

    // Move the paddles.
    // TODO(scotttodd): Player input to control paddles, beatable AI.
    int y = math::Clamp(static_cast<int>(ball_y_position_) - PADDLE_HEIGHT / 2,
                        0, PLAY_HEIGHT - PADDLE_HEIGHT);
    left_paddle_rect_.origin.y = y;
    right_paddle_rect_.origin.y = y;
    MoveWindowLeftPaddle();
    MoveWindowRightPaddle();

    // Simple bounce against walls.
    // TODO(scotttodd): continuous collision detection / physics
    if (ball_x_position_ + ball_rect_.size.width > PLAY_WIDTH) {
      // TODO(scotttodd): Add point for player 1, start next round.
      ball_x_velocity_ = -std::abs(ball_x_velocity_);
    }
    if (ball_x_position_ < 0) {
      // TODO(scotttodd): Add point for player 2, start next round.
      ball_x_velocity_ = std::abs(ball_x_velocity_);
    }
    if (ball_y_position_ + ball_rect_.size.height > PLAY_HEIGHT) {
      ball_y_velocity_ = -std::abs(ball_y_velocity_);
    }
    if (ball_y_position_ < 0) {
      ball_y_velocity_ = std::abs(ball_y_velocity_);
    }

    // Simple bounce against paddles.
    // If the ball overlaps with a paddle, simply redirect to the other side.
    if (rects_intersect(ball_rect_, left_paddle_rect_)) {
      ball_x_velocity_ = std::abs(ball_x_velocity_);
    }
    if (rects_intersect(ball_rect_, right_paddle_rect_)) {
      ball_x_velocity_ = -std::abs(ball_x_velocity_);
    }

    ball_rect_.origin.x = ball_x_position_;
    ball_rect_.origin.y = ball_y_position_;
    MoveWindowBall();

    last_tick_time_ = now;
  }

 private:
  void OnError(ref_ptr<Control> target) override {
    LOG(INFO) << "Window error, ending game";
    done_event_->Set();
  }

  void OnDestroyed(ref_ptr<Control> target) override {
    LOG(INFO) << "A window was closed, ending game";
    done_event_->Set();
  }

  void OnResized(ref_ptr<Control> target, Rect2D bounds) override {
    // TODO(scotttodd): Update game state or override window positions.
  }

  MessageLoop::TaskList task_list_;
  ref_ptr<MessageLoop> message_loop_;
  ref_ptr<Event> done_event_;

  ref_ptr<Window> window_background_;
  ref_ptr<Window> window_left_paddle_;
  ref_ptr<Window> window_right_paddle_;
  ref_ptr<Window> window_ball_;

  Rect2D ball_rect_;
  Rect2D left_paddle_rect_;
  Rect2D right_paddle_rect_;

  double last_tick_time_ = 0.0;

  // Ball speed in screen pixels / second.
  float ball_speed_ = 1500.0f;
  float ball_x_position_ = 0.0f;
  float ball_y_position_ = 0.0f;
  float ball_x_velocity_ = 0.0f;
  float ball_y_velocity_ = 0.0f;
};

int DemoEntryPoint(int argc, char** argv) {
  auto demo = make_unique<WindowPongDemo>();
  demo->Run();  // Blocks until the demo is exited.
  demo.reset();
  LOG(INFO) << "Clean exit!";
  return 0;
}

}  // namespace
}  // namespace ui
}  // namespace xrtl

DECLARE_ENTRY_POINT(xrtl::ui::DemoEntryPoint);
