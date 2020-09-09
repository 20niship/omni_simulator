#include "Simulator.h"


int main(int argc, char *argv[]){

    try {
        Simulator::init(argc, argv);
        //↓初期化時に一度だけ実行させたいことはここに書く

        //電流制御のパラメタを設定(380のパラメタ）
        //制限電流は5000mA
        Simulator::setMotorParams(0, 900, 1500, 1024, 5000);
        Simulator::setMotorParams(1, 900, 1500, 1024, 5000);

        Simulator::loop();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception" << std::endl;
    }
    Simulator::quit();
    return 0;
}

void control(int last_command){
    // dReal tmp = dJointGetAMotorAngle(wheel[0].motor, 1);
    // dReal u   = kp * (target - tmp);

    switch(last_command){
        case 'f':
        std::cout << "[command] = F (forward) \n";
        Simulator::setMotorCurrent(0, 10.0);
        Simulator::setMotorCurrent(1, -10.0);
        Simulator::setMotorCurrent(2, 0.0);
        break;

        case 's':
        std::cout << "[command] = S (stop) \n";
        Simulator::setMotorCurrent(0, 0.0);
        Simulator::setMotorCurrent(1, 0.0);
        Simulator::setMotorCurrent(2, 0.0);
        break;

        case 'r':
        std::cout << "[command] = R (rotation) \n";
        Simulator::setMotorCurrent(0, 10.0);
        Simulator::setMotorCurrent(1, 10.0);
        Simulator::setMotorCurrent(2, 10.0);
        break;

    }
}



