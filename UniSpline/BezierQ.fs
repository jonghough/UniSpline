namespace UniSpline
open UnityEngine

///<summary>
/// Implementation of a quadratic bezier curve using Vector3 points for the two end points and the
/// control point.
///</summary>
type BezierQ(P0 : Vector3, P1 : Vector3, P2 : Vector3, limit : float32 , onFinish : System.Action) = 
    let mutable t :float32 = 0.0f
    member private this.l : float32 = limit
    member private this.p0 : Vector3 = P0
    member private this.p1 : Vector3 = P1
    member private this.p2 : Vector3 = P2
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
            this.p0*(1.0f-nt)*(1.0f-nt)+this.p1*2.0f*nt*(1.0f-nt)+this.p2*nt*nt

    ///<summary>
    /// Returns the normalized tangent vector of the bezier curve at the current time.
    /// If time is less than or equal to zero or greater than the time limit, then will
    /// return the zero vector.
    ///</summary>
    member public this.GetTangent () =
        if t > this.l then Vector3.zero
        elif t < 0.0f then Vector3.zero
        else 
        let nt = t / this.l
        let s = 1.0f - nt
        (-2.0f * s * this.p0 + 2.0f * (s - nt) * this.p1 + 2.0f * nt * this.p2).normalized