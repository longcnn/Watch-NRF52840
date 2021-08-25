#pragma once

namespace Watch{
  namespace System {
      enum class Messages {
        GoToSleep, 
        GoToRunning, 
        BleConnected, 
        TouchWakeUp, 
        OnChargingEvent,
        OnNewNotification,
        BleFirmwareUpdateStarted, 
        BleFirmwareUpdateFinished, 
        OnTouchEvent, OnButtonEvent, 
        OnDisplayTaskSleeping, 
        AlwaysDisplay,
        UpdateTimeOut};
    };
}

