import numpy as np

def normalize(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("Zero vector cannot be normalized.")
    return v / n

def plate_ball_joint_positions(n, h, R):
    """
    원하는 플랫폼 법선 n=(a,b,c)로부터
    세 개의 볼조인트 위치 P_k를 계산.

    반환:
        P: (3,3) array, 각 행이 [x,y,z]
    """
    n = normalize(n)

    phis = np.array([np.pi, np.pi/3, 5*np.pi/3])
    C = np.array([0.0, 0.0, h])

    P = []
    for phi in phis:
        # 모터 평면의 법선벡터
        # (이 부호 선택은 앞에서 설명한 convention)
        n_k = np.array([-np.sin(phi), np.cos(phi), 0.0])

        # 두 평면의 교선 방향
        d = np.cross(n_k, n)
        d = normalize(d)

        # C에서 R만큼 간 점이 볼조인트
        Pk = C + R * d
        P.append(Pk)

    return np.array(P)

def planar_2r_ik(x, z, l1, l2, elbow="out"):
    """
    평면 2R 역기구학.
    x: 모터에서 볼조인트까지의 수평거리(모터 평면 내)
    z: 모터에서 볼조인트까지의 높이
    elbow: "out" 또는 "in"
    
    반환:
        q1: 첫 번째 관절각 (바닥 기준)
        q2: 두 번째 관절각
    """
    D = np.sqrt(x**2 + z**2)

    # 도달 가능성 검사
    cos_q2 = (- D**2 + l1**2 + l2**2) / (2.0 * l1 * l2)
    if cos_q2 < -1.0 or cos_q2 > 1.0:
        raise ValueError(f"Unreachable point: D={D:.6f}, cos(q2)={cos_q2:.6f}")

    sin_term = np.sqrt((1 - cos_q2) * (1 + cos_q2))
    if elbow == "out":
        sin_q2 = sin_term
    else:
        raise ValueError("elbow must be 'out' or 'in'")

    q2 = np.arctan2(sin_q2, cos_q2)
    
    A = l1 - l2 * cos_q2
    B = l2 * sin_q2
    
    q1 = np.arctan2(z * A + x * B, x * A - z * B)

    return q1, q2


def motor_angles_from_normal(n, h, R, r_base, l1, l2, elbow="out"):
    """
    플랫폼 법선 n으로부터 각 모터의 관절각 계산.

    반환:
        angles: list of dict
            각 원소는 {
                'phi': 모터 방위각,
                'P': 볼조인트 위치,
                'q1': 첫 번째 관절각,
                'q2': 두 번째 관절각
            }
    """
    n = normalize(n)

    phis = np.array([np.pi, np.pi/3, 5*np.pi/3])
    M = np.array([[r_base*np.cos(phi), r_base*np.sin(phi), 0.0] for phi in phis])

    P = plate_ball_joint_positions(n, h, R)

    results = []
    for k, phi in enumerate(phis):

        v = P[k] - M[k]
        z_local = v[2]             # 높이 성분
        x_local = np.sqrt((v[0])**2 + (v[1])**2)
        
        # 2R IK
        q1, q2 = planar_2r_ik(x_local, z_local, l1, l2, elbow=elbow)

        results.append({
            "phi": phi,
            "P": P[k],
            "q1": q1,   # 바닥과 첫 링크가 이루는 각
            "q2": q2,    # 엘보 각
        })

    return results

# ------------------------------
# 사용 예시
# ------------------------------
if __name__ == "__main__":
    n = np.array([0.198997487, 0, 0.98])   # 원하는 법선벡터
    h = 20                         # 플랫폼 중심 높이
    R = 10                          # 중심~볼조인트 거리
    r_base = 10                    # 모터 배치 반지름
    l1 = 7
    l2 = 20

    angles = motor_angles_from_normal(n, h, R, r_base, l1, l2, elbow="out")
    
    for i, a in enumerate(angles):
        print(f"Motor {i}:")
        print(f"  P  = {a['P']}")
        print(f"  q1 = {np.degrees(a['q1']):.5f} deg")
        print(f"  q2 = {np.degrees(a['q2']):.5f} deg")

    