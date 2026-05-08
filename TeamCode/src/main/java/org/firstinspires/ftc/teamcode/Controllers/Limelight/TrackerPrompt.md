修改Tracker.java：
有一个参数hasTarget，用于判断是否有目标。
有一个参数targetTheta，用于存储目标角度，单位：弧度。
从Detector中获取hasTarget()和bearing()方法，用于判断是否有目标和获取目标角度。
对Detector的输出低通滤波，若目标角度消失或大幅变化（超过BearingThreshold）一定帧数后再修改targetTheta。
完成后修改其他使用Tracker的程序，改从getTargetTheta()方法获取目标角度。