namespace UniSpline


open UnityEngine
open System
open System.Collections
open System.Collections.Generic
module CRS =

    ///<summary>
    /// Catmull Rom spline implementation for GameObject control point list. Using GameObject references
    /// allows manipulation of the control point positions while the spline curve is being traversed.
    ///</summary>
    type CatmullRomAdvanced ( ptList : List<GameObject>, timeLimit : float32, onFinish : System.Action )=
        let mutable t : float32 = 0.0f
        let mutable mt : float32 = 0.0f
        let mutable currentIndex : int32 = 0
        member private this.tArray : List<GameObject> = ptList
        member private this.timeLimit : float32 = timeLimit
        member private this.onFinish : System.Action = onFinish

        
        member public this.UpdateCurve( dt : float32) : Vector3 = 
            t <- t + dt
           // let mutable v : Vector3 = Vector3.zero
            if t > this.timeLimit then 
                onFinish.Invoke()

            let index = 1 + Mathf.Min((int)((float32)(this.tArray.Count - 3) * t / this.timeLimit), this.tArray.Count - 4)
            mt <- mt + dt *(float32) (this.tArray.Count - 3 ) * 1.0f / timeLimit

            let mutable tt = mt
            if index > currentIndex then 
                currentIndex <- index
                mt <- 0.0f

            tt <- mt
            let p0 : Vector3 = this.tArray.[index-1].transform.position
            let p1 : Vector3 = this.tArray.[index].transform.position
            let p2 : Vector3 = this.tArray.[index+1].transform.position
            let p3 : Vector3 = this.tArray.[index+2].transform.position

            0.5f*((2.0f*p1)+((p2-p0)*tt)+((2.0f*p0-5.0f*p1+4.0f*p2-p3)*tt*tt)+(3.0f*p1-p0-3.0f*p2+p3)*tt*tt*tt)


        member public this.UpdateCurveAtSpeed( dt : float32, speed : float32) : Vector3 =
            let mutable spd = 2.0f * speed * this.timeLimit / (float32)(this.tArray.Count - 2)
            spd <- spd * 1.0f
            if System.Double.IsNaN((float)spd) then 
                t <- timeLimit
                this.tArray.[this.tArray.Count - 2].transform.position
            else
                if spd < 0.00001f then spd <- 0.00001f
                let realDT = dt * speed / spd
                this.UpdateCurve(realDT)


        member public this.GetTangentAtCurrentTime() =
            if t < 0.0f || t > this.timeLimit then Vector3.zero
            else
                let p0 : Vector3 = this.tArray.[currentIndex-1].transform.position
                let p1 : Vector3 = this.tArray.[currentIndex].transform.position
                let p2 : Vector3 = this.tArray.[currentIndex+1].transform.position
                let p3 : Vector3 = this.tArray.[currentIndex+2].transform.position
                (p2-p0)+2.0f*(2.0f*p0-5.0f*p1+4.0f*p2-p3)*mt+3.0f*(3.0f*p1-p0-3.0f*p2+p3)*mt*mt

        member public this.Get2ndDerivativeAtCurrentTime() = 
            if t < 0.0f || t > this.timeLimit then Vector3.zero
            else
                let p0 : Vector3 = this.tArray.[currentIndex-1].transform.position
                let p1 : Vector3 = this.tArray.[currentIndex].transform.position
                let p2 : Vector3 = this.tArray.[currentIndex+1].transform.position
                let p3 : Vector3 = this.tArray.[currentIndex+2].transform.position
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