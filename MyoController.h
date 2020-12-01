#pragma once
#include <myo/myo.hpp>

class MyoController : public myo::DeviceListener
{
public:
    int count = 0;
    std::vector<std::vector<int>> CurrentEMG;
    int averageEMG[8];
    enum MyoState
    {
        Up,
        Down,
        Left,
        Right,
        Rest
    };
    MyoState Direction = MyoState::Rest;
    MyoController()
        : onArm(false), isUnlocked(false), currentPose()
    {
    }
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp);
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp);

    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp);

    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
        myo::WarmupState warmupState);

    void onUnpair(myo::Myo* myo, uint64_t timestamp);


    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;

    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    // These values are set by onPose() above.
    myo::Pose currentPose;
};