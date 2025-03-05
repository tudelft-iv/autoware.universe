// File: g920_joy_converter.hpp
#ifndef AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__G920_JOY_CONVERTER_HPP_
#define AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__G920_JOY_CONVERTER_HPP_

#include "autoware/joy_controller/joy_converter/joy_converter_base.hpp"

#include <cstdlib>
#include <complex>

namespace autoware::joy_controller
{
class G920JoyConverter : public JoyConverterBase
{
public:
        explicit G920JoyConverter(const sensor_msgs::msg::Joy & j) : j_(j) {}
        
        //float accel() const{ return (AccelPedal()- 1.0)/2; }
        //float brake() const{ return (BrakePedal()- 1.0)/2; }
        
        float accel() const
        {
        constexpr float eps = 0.000001;
        if (std::fabs(AccelPedal()) < eps) {
        return 0.0f;
        }
        return (AccelPedal() + 1.0f)/2; //-1.0)/2
        }


        float brake() const
        {
        constexpr float eps = 0.000001;
        if (std::fabs(BrakePedal()) < eps) {
        return 0.0f;
        }
        return (BrakePedal() + 1.0f)/2 ; //-1.0)/2
        }

        float steer() const { return Steer(); }

        bool shift_up() const { return CursorUpDown() == 1.0f; }
        bool shift_down() const { return CursorUpDown() == -1.0f; }
        bool shift_drive() const { return CursorLeftRight() == 0.0f; }
        bool shift_reverse() const { return CursorLeftRight() == -1.0f; }

        
        bool turn_signal_left() const { return button_6(); }
        bool turn_signal_right() const { return button_5(); }
        bool clear_turn_signal() const { return button_4(); }

        bool gate_mode() const { return button_3(); }

        bool emergency_stop() const { return !reverse() && button_11(); }
        bool clear_emergency_stop() const { return reverse() && button_11(); }

        bool autoware_engage() const { return !reverse() && button_1(); }
        bool autoware_disengage() const { return reverse() && button_1(); }

        bool vehicle_engage() const { return !reverse() && button_2(); }
        bool vehicle_disengage() const { return reverse() && button_2(); }



private:

        //G920 Axis Mapping
        float Steer() const { return j_.axes.at(0); }
        float AccelPedal() const { return j_.axes.at(1); }
        float BrakePedal() const { return j_.axes.at(2); }
        

        float CursorLeftRight() const { return j_.axes.at(4); }
        float CursorUpDown() const { return j_.axes.at(1); }

        //float L2() const { return j_.axes.at(6); }
        //float R2() const { return j_.axes.at(7); }


        //G920 button mapping

        bool button_1() const { return j_.buttons.at(0); } 
        bool button_2() const { return j_.buttons.at(1); }
        bool button_3() const{ return j_.buttons.at(2); }
        bool button_4() const {return j_.buttons.at(3); }
        bool button_5() const {return j_.buttons.at(4); }
        bool button_6() const {return j_.buttons.at(5); }
        bool button_7() const {return j_.buttons.at(6); }
        bool button_8() const {return j_.buttons.at(7); }
        bool button_9() const {return j_.buttons.at(8); }
        bool button_10() const {return j_.buttons.at(9); }
        bool button_11() const {return j_.buttons.at(10); }

        const sensor_msgs::msg::Joy j_;
        

        

        bool reverse() const { return button_7(); }

        

};
}// namespace autoware::joy_controller

#endif  // AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__G920_JOY_CONVERTER_HPP_
