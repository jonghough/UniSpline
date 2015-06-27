namespace UniSpline
open UnityEngine

module BezierC =


    ///<summary>
    /// Implementation of a cubic bezier curve using Vector3 points for the two end points and the
    /// control point.
    ///</summary>
    type BezierC(P0 : Vector3, P1 : Vector3, P2 : Vector3, P3 : Vector3, limit : float32 , onFinish : System.Action) = 
        let mutable t :float32 = 0.0f
        member private this.l : float32 = limit
        member private this.p0 : Vector3 = P0
        member private this.p1 : Vector3 = P1
        member private this.p2 : Vector3 = P2
        member private this.p3 : Vector3 = P3
        member private this.onFinish : System.Action = onFinish

   
       ///<summary>
       /// Updates the Bezier curve and returns the next point (Vector3)
       ///</summary>
        member public this.Update(dt : float32) =
            let t = t + dt
            if t > this.l then 
                onFinish.Invoke()
                Vector3.zero
            else 
                let nt = t / this.l
                this.p0*(1.0f-nt)*(1.0f-nt)*(1.0f-nt)+this.p1*3.0f*nt*(1.0f-nt)*(1.0f-nt)+3.0f*this.p2*nt*nt*(1.0f-nt)+this.p3*nt*nt*nt

        