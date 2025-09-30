using UnityEngine;

/// <summary>
/// Elliptical (Keplerian) orbit around a focus (the primary body).
/// Place this on the moon. Assign `primary` to your planet transform.
/// </summary>
[ExecuteAlways]
public class RotateAround : MonoBehaviour
{
    [Header("Primary / Focus")]
    public Transform primary;

    [Header("Shape (a,e)")]
    [Min(0.01f)] public float semiMajorAxis = 10f; // a
    [Range(0f, 0.999f)] public float eccentricity = 0.3f; // e

    [Header("Timing")]
    [Min(0.01f)] public float orbitalPeriod = 20f; // seconds per full orbit
    public float timeScale = 1f;                    // multiply time flow (fast-forward/slow-mo)
    public float meanAnomalyAtEpochDeg = 0f;        // starting position (0 = periapsis)
    public float epochSeconds = 0f;                 // t0 reference (seconds); leave 0 for now

    [Header("Orientation (degrees)")]
    public float longitudeOfAscendingNode = 0f;     // Ω
    public float inclination = 0f;                  // i
    public float argumentOfPeriapsis = 0f;          // ω

    [Header("Gizmos")]
    public bool drawOrbitPath = true;
    [Range(32, 512)] public int gizmoSegments = 128;
    public float gizmoPointSize = 0.05f;

    // Internal time accumulator (keeps Update deterministic with timeScale)
    private float _simTime;

    void OnEnable()
    {
        // Initialize sim time so the start position matches meanAnomalyAtEpochDeg
        _simTime = epochSeconds;
        UpdatePosition();
    }

    void Update()
    {
        if (!Application.isPlaying)
        {
            // Keep editor previews live
            UpdatePosition();
            return;
        }

        _simTime += Time.deltaTime * Mathf.Max(0f, timeScale);
        UpdatePosition();
    }

    void UpdatePosition()
    {
        if (primary == null) return;

        // 1) Mean motion (rad/s) and mean anomaly
        float n = 2f * Mathf.PI / orbitalPeriod; // mean motion
        float M0 = meanAnomalyAtEpochDeg * Mathf.Deg2Rad;
        float M = NormalizeAngleRad(M0 + n * (_simTime - epochSeconds));

        // 2) Solve Kepler's equation for eccentric anomaly E: M = E - e * sin(E)
        float e = eccentricity;
        float E = SolveEccentricAnomaly(M, e);

        // 3) True anomaly ν and orbital radius r
        float cosE = Mathf.Cos(E);
        float sinE = Mathf.Sin(E);
        float r = semiMajorAxis * (1f - e * cosE);

        float sqrt1pe = Mathf.Sqrt(1f + e);
        float sqrt1me = Mathf.Sqrt(1f - e);
        float nu = 2f * Mathf.Atan2(sqrt1pe * Mathf.Sin(E * 0.5f),
                                    sqrt1me * Mathf.Cos(E * 0.5f));

        // 4) Position in the orbital plane (perifocal frame), use XZ plane in Unity
        Vector3 perifocal = new Vector3(r * Mathf.Cos(nu), 0f, r * Mathf.Sin(nu));

        // 5) Rotate from perifocal to world using Ω, i, ω (Rz(Ω) * Rx(i) * Rz(ω))
        Quaternion rot =
            Quaternion.AngleAxis(longitudeOfAscendingNode, Vector3.up) *
            Quaternion.AngleAxis(inclination, Vector3.right) *
            Quaternion.AngleAxis(argumentOfPeriapsis, Vector3.up);

        Vector3 worldPos = primary.position + rot * perifocal;

        transform.position = worldPos;

        // (Optional) orient the moon to face the primary
        // transform.rotation = Quaternion.LookRotation(primary.position - transform.position, Vector3.up);
    }

    // Newton-Raphson solve for E
    float SolveEccentricAnomaly(float M, float e)
    {
        // Good initial guess
        float E = (e < 0.8f) ? M : Mathf.PI;
        // Iterate
        for (int i = 0; i < 8; i++)
        {
            float f = E - e * Mathf.Sin(E) - M;
            float fPrime = 1f - e * Mathf.Cos(E);
            E -= f / (fPrime + 1e-8f);
        }
        return NormalizeAngleRad(E);
    }

    float NormalizeAngleRad(float a)
    {
        a = Mathf.Repeat(a, 2f * Mathf.PI);
        return a;
    }

    void OnDrawGizmos()
    {
        if (!drawOrbitPath || primary == null) return;

        // Draw sampled ellipse in world space
        Gizmos.color = new Color(1f, 1f, 1f, 0.5f);

        Quaternion rot =
            Quaternion.AngleAxis(longitudeOfAscendingNode, Vector3.up) *
            Quaternion.AngleAxis(inclination, Vector3.right) *
            Quaternion.AngleAxis(argumentOfPeriapsis, Vector3.up);

        Vector3 prev = Vector3.zero;
        for (int s = 0; s <= gizmoSegments; s++)
        {
            float t = (float)s / gizmoSegments;
            float E = t * 2f * Mathf.PI;

            float r = semiMajorAxis * (1f - eccentricity * Mathf.Cos(E));
            float cosE = Mathf.Cos(E);
            float sinE = Mathf.Sin(E);

            // True anomaly from E for a smoother shape at high e
            float nu = 2f * Mathf.Atan2(
                Mathf.Sqrt(1f + eccentricity) * Mathf.Sin(E * 0.5f),
                Mathf.Sqrt(1f - eccentricity) * Mathf.Cos(E * 0.5f));

            Vector3 perifocal = new Vector3(r * Mathf.Cos(nu), 0f, r * Mathf.Sin(nu));
            Vector3 wp = primary.position + rot * perifocal;

            if (s > 0) Gizmos.DrawLine(prev, wp);
            prev = wp;

            // Tiny points along the path to help visualize
            if (gizmoPointSize > 0f)
            {
                Gizmos.DrawSphere(wp, gizmoPointSize);
            }
        }

        // Mark periapsis point
        float rPeri = semiMajorAxis * (1f - eccentricity);
        Vector3 peri = primary.position + rot * new Vector3(rPeri, 0f, 0f);
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(peri, gizmoPointSize * 2f);
    }
}
