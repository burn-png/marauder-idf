#pragma once
namespace marauder::ui {
class Widget { public: virtual ~Widget() = default; virtual void paint() = 0; };
class View : public Widget { public: virtual void on_enter() {} virtual void on_leave() {} };
class SpectrumView final : public View {
public:
    void paint() override;
    void on_enter() override;
};
}
