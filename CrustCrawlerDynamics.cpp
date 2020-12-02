#include "CrustCrawlerDynamics.h"
SimpleSerial serial("COM6", 115200);
void CrustCrawlerDynamics::errortheta()
{
    for (int i = 0; i < 4; i++)
    {
        err[i] = Thetaref[i] - angle[i];
    }
}

void CrustCrawlerDynamics::errordtheta()
{
   
    for (int i = 0; i < 4; i++)
    {
        
        derr[i] = dThetaref[i] - anglevelocity[i];
       // std::cout << "derr " << i << " " << derr[i] << " ";
    }
    //std::cout << std::endl;
}

void CrustCrawlerDynamics::UpdatePos()
{
    
    bool messageReceived = false;
    bool messageDone = false;
    bool start = false;
    std::string message;
    message = serial.readLine();
    std::string number;
    std::string vnumber;
    if (message != "") {
        for (int i = 0; i < message.length(); i++)
        {
            if (message[i] == 'A') {
                std::string temp = "";
                temp += message[i + 1];

                for (int j = i + 2; j < message.length(); j++)
                {
                    if (message[j] == 'A' || message[j] == 'V') {
                        j = message.length();
                        angle[stoi(temp)] = stoi(number);

                        number = "";
                    }
                    else if ((message[j] >= '0' && message[j] <= '9') || message[j] == '-') {

                        number += message[j];
                    }
                }
            }
            if (message[i] == 'V') {
                std::string temp = "";
                temp += message[i + 1];

                for (int j = i + 2; j < message.length(); j++)
                {
                    if (message[j] == 'A' || message[j] == 'V' || message[j] == 'M') {
                        j = message.length();
                        anglevelocity[stoi(temp)] = stoi(number);
                        number = "";
                    }
                    else if ((message[j] >= '0' && message[j] <= '9') || message[j] == '-') {

                        number += message[j];
                    }
                }
            }
        }

    }


}

void CrustCrawlerDynamics::updateref(float theta[4], float dtheta[4], float ddtheta[4])
{
    Thetaref[0] = theta[0];
    dThetaref[0] = dtheta[0];
    ddThetaref[0] = ddtheta[0];

    Thetaref[1] = theta[1];
    dThetaref[1] = 0;
    ddThetaref[1] = 0;

    Thetaref[2] = theta[2];
    dThetaref[2] = 0;
    ddThetaref[2] = 0;

    Thetaref[3] = theta[3];
    dThetaref[3] = dtheta[3];
    ddThetaref[3] = ddtheta[3];
}

void CrustCrawlerDynamics::updatem()
{
    m[0] = (0.0027 * cos(angle[2] + angle[3] - 0.3717) - 0.0027 * cos(2.0 * angle[1] + angle[2] + angle[3] - 0.3717) - 0.0116 * cos(2.0 * angle[1] - 0.0049) - 0.0026 * cos(2.0 * angle[1] + 2.0 * angle[2] - 0.0083) - 0.0005 * cos(2.0 * angle[1] + 2.0 * angle[2] + 2.0 * angle[3] - 0.7434) + 0.0018 * cos(angle[3] - 0.3717) + 0.0083 * cos(angle[2] - 0.0072) - 0.0018 * cos(2.0 * angle[1] + 2.0 * angle[2] + angle[3] - 0.3717) - 0.0083 * cos(2.0 * angle[1] + angle[2] - 0.0072) + 0.0150);
    m[1] = (0.0053 * cos(angle[2] + angle[3] - 0.3717) + 0.0036 * cos(angle[3] - 0.3717) + 0.0165 * cos(angle[2] - 0.0072) + 0.0273);
    m[2] = (0.0036 * cos(angle[3] - 0.3717) + 0.0062);
    m[3] = (0.0010);
}

void CrustCrawlerDynamics::updateg()
{
    g[0] = 0;
    g[1] = (0.3683 * cos(angle[1] + angle[2] + 1.5636) + 0.1190 * cos(angle[1] + angle[2] + angle[3] + 0.1991) + 1.0545 * cos(angle[1] + 1.5686));
    g[2] = (0.3683 * cos(angle[1] + angle[2] + 1.5636) + 0.1190 * cos(angle[1] + angle[2] + angle[3] + 0.1991));
    g[3] = (0.1190 * cos(angle[1] + angle[2] + angle[3] + 0.1991));

}

void CrustCrawlerDynamics::send_torque()
{
    PWM[0] = 191 * torque[0] + 124 * dThetaref[0];
    PWM[1] = 107 * torque[1] + 166 * dThetaref[1];
    PWM[2] = 191 * torque[2] + 124 * dThetaref[2];
    PWM[3] = 191 * torque[3] + 146 * dThetaref[3];
    int pwm[4];
    pwm[0] = PWM[0];
    pwm[1] = PWM[1];
    pwm[2] = PWM[2];
    pwm[3] = PWM[3];

    std::string message;

    message = "P";
    message += std::to_string(0);
    message += std::to_string(pwm[0]);
    message += ':';

    message += "P";
    message += std::to_string(1);
    message += std::to_string(pwm[1]);
    message += ':';
    message += "P";
    message += std::to_string(2);
    message += std::to_string(pwm[2]);
    message += ':';

    message += "P";
    message += std::to_string(3);
    message += std::to_string(pwm[3]);
    message += ':';


   // std::cout << message << std::endl;
    serial.writeString(message);

}

void CrustCrawlerDynamics::control(float theta[4], float dtheta[4], float ddtheta[4])
{
    
    updateref(theta, dtheta, ddtheta);
    UpdatePos();
    for (int i = 0; i < 4; i++) {
        // std::cout << i << " " << angle[i] << " ";
        anglevelocity[i] *= 3.14 / 180;
        angle[i] *= 3.14 / 180;
    }
    angle[1] -= 3.14;
    angle[2] -= 3.14;
    angle[3] -= 3.14;
   
   // std::cout << std::endl;
    errortheta();
    errordtheta();
    updatem();
    updateg();
    for (int i = 0; i < 4; i++) {
        torque[i] = m[i] * (ddThetaref[i] + (kp[i] * err[i]) + (kd[i] * derr[i])) + g[i];
        //std::cout << "derr " << i << " " << derr[i];
    }
   // std::cout << std::endl;

    send_torque();
}

