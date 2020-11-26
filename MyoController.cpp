#include "MyoController.h"

void MyoController::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
    currentPose = pose;
    if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
        // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
        // Myo becoming locked.
        myo->unlock(myo::Myo::unlockHold);
        // Notify the Myo that the pose has resulted in an action, in this case changing
        // the text on the screen. The Myo will vibrate.
        myo->notifyUserAction();
    }
    else {
        // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
        // are being performed, but lock after inactivity.
        myo->unlock(myo::Myo::unlockTimed);
    }
}

void MyoController::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
    if (count < 20) {
        std::vector<int> temp;
        for (int i = 0; i < 8; i++)
        {

            temp.push_back(abs(static_cast<long int>(emg[i])));
        }
        CurrentEMG.push_back(temp);
        count++;
    }
    else if (CurrentEMG.size() == 20) {
        std::vector<int> temp;
        for (int i = 0; i < 8; i++)
        {
            int sum = 0;
            for (int j = 0; j < 20; j++)
            {
                sum += CurrentEMG[j][i];
            }
            averageEMG[i] = sum / 20;
            temp.push_back(abs(static_cast<long int>(emg[i])));


        }

        CurrentEMG.push_back(temp);
        CurrentEMG.erase(CurrentEMG.begin());
        float up = averageEMG[4] * 1.5 / (1 + averageEMG[1] + averageEMG[7]);
        float right = averageEMG[3] * 2.7 / (1 + averageEMG[7] + averageEMG[1]);
        float left = (averageEMG[0] + averageEMG[7]) / (averageEMG[5] + 1);
        float down = (1 * (averageEMG[0] + averageEMG[1] + averageEMG[2])) / (1 + averageEMG[5] + averageEMG[6]);
        float rest = (averageEMG[4] + averageEMG[5] + averageEMG[6] + averageEMG[7]) / 20;
        if (up + right + left + down <= 5) {
            Direction = Rest;
        }
        else if (up > right && up > left && up > down)
        {
            Direction = Up;
        }
        else if (left > right && left > up && left > down)
        {
            Direction = Left;
        }
        else if (right > left && right > up && right > down)
        {
            Direction = Right;
        }
        else if (down > right && down > left && down > up)
        {
            Direction = Down;
        }
    }
}

void MyoController::onLock(myo::Myo* myo, uint64_t timestamp)
{
    
    isUnlocked = false;
}

void MyoController::onUnlock(myo::Myo* myo, uint64_t timestamp)
{
    isUnlocked = true;
}

void MyoController::onArmUnsync(myo::Myo* myo, uint64_t timestamp)
{
    onArm = false;
}

void MyoController::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation, myo::WarmupState warmupState)
{
    onArm = true;
    whichArm = arm;
}

void MyoController::onUnpair(myo::Myo* myo, uint64_t timestamp)
{
    onArm = false;
    isUnlocked = false;
}
