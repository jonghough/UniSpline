namespace UniSpline
open UnityEngine
open System

///<summary>
/// Implementation of Catmull-Rom spline using an array of Vector3 contorl points.
///</summary>
type CatmullRomSpline(controlPoints : Vector3[], timeLimit : float32, onFinish : Action) = 
    let mutable t : float32 = 0.0f
    let mutable mt : float32 = 0.0f
    let mutable currentIndex : int32 = 0
    member private this.controlPts : Vector3[] = controlPoints
    member private this.timeLimit : float32 = timeLimit
    member private this.onFinish : Action = onFinish

    ///<summary>
    /// Updates the curve and returns the next Vector3 point on the curve.
    ///</summary>
    member public this.UpdateCurve(dt : float32) = 
        t <- t + dt
        let mutable v = Vector3.zero
        if t < 0.0f then v <- this.controlPts.[1]
        elif t > this.timeLimit then 
            onFinish.Invoke()
            v<-this.controlPts.[this.controlPts.Length - 1]

        let index = 1 + Mathf.Min((int)((float32)(this.controlPts.Length - 3) * t / this.timeLimit), this.controlPts.Length - 4)
        mt <- mt + dt *(float32) (this.controlPts.Length - 3 ) * 1.0f / timeLimit

        let mutable tt = mt
        if index > currentIndex then 
            currentIndex <- index
            mt <- 0.0f

        tt <- mt
        let p0 = this.controlPts.[index - 1]
        let p1 = this.controlPts.[index]
        let p2 = this.controlPts.[index + 1]
        let p3 = this.controlPts.[index + 2]

        0.5f*((2.0f*p1)+((p2-p0)*tt)+((2.0f*p0-5.0f*p1+4.0f*p2-p3)*tt*tt)+(3.0f*p1-p0-3.0f*p2+p3)*tt*tt*tt)


    member public this.UpdateCurveAtSpeed( dt : float32, speed : float32) =
        let mutable spd = 2.0f * speed * this.timeLimit / (float32)(this.controlPts.Length - 2)
        spd <- spd * 1.0f
        if System.Double.IsNaN((float)spd) then 
            t <- timeLimit
            this.controlPts.[this.controlPts.Length - 2]
        else
            if spd < 0.00001f then spd <- 0.00001f
            let realDT = dt * speed / spd
            this.UpdateCurve(realDT)


    member public this.GetTangentAtCurrentTime() =
        if t < 0.0f || t > this.timeLimit then Vector3.zero
        else
            let p0 = this.controlPts.[currentIndex - 1]
            let p1 = this.controlPts.[currentIndex]
            let p2 = this.controlPts.[currentIndex + 1]
            let p3 = this.controlPts.[currentIndex + 2]
            (p2-p0)+2.0f*(2.0f*p0-5.0f*p1+4.0f*p2-p3)*mt+3.0f*(3.0f*p1-p0-3.0f*p2+p3)*mt*mt

    member public this.Get2ndDerivativeAtCurrentTime() = 
        if t < 0.0f || t > this.timeLimit then Vector3.zero
        else
            let p0 = this.controlPts.[currentIndex - 1]
            let p1 = this.controlPts.[currentIndex]
            let p2 = this.controlPts.[currentIndex + 1]
            let p3 = this.controlPts.[currentIndex + 2]
            2.0f*(2.0f*p0-5.0f*p1+4.0f*p2-p3)+6.0f*(3.0f*p1-p0-3.0f*p2+p3)*mt

    member public this.GetCurvature() =
        let t1 = this.GetTangentAtCurrentTime ()
        let t2 = this.Get2ndDerivativeAtCurrentTime ()
        let t1Mag = t1.magnitude
        if t1.magnitude < 0.001f || t2.magnitude < 0.001f then 0.0f
        elif Double.IsNaN((float)t1Mag) || Double.IsInfinity((float)t1Mag) then 0.0f
        else Vector3.Cross(t1,t2).magnitude*1.0f/(t1Mag*t1Mag*t1Mag)

    member public this.GetBinormal () =
        Vector3.Cross ( this.GetTangentAtCurrentTime (), this.Get2ndDerivativeAtCurrentTime () )

    member public this.GetNormal () =
        Vector3.Cross(this.GetBinormal(), this.GetTangentAtCurrentTime().normalized).normalized