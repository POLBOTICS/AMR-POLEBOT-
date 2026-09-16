"""Parameter fisik robot Polebot dipetakan ke simbol model jurnal Alipour 2019.

Semua nilai diturunkan dari SDF (polebot_amr_description.sdf, trolley nested).
Dipakai oleh derivasi H(q̄)/C̃ (journal_dynamics.py) dan controller jurnal.

Notasi mengikuti Fig.7 & Tabel 2 jurnal:
  subskrip 0 = TRAKTOR, tanpa subskrip = TRAILER
  b  = ½ jarak antar roda (half track)
  d  = jarak CG → poros roda aktif
  r  = radius roda
  L0 = jarak poros roda traktor → pin hitch (J)
  L  = jarak CG trailer → pin hitch (J)
  d' = L + d = jarak pin hitch → poros roda trailer (titik kontrol P)
"""
import math

# ── TRAKTOR ───────────────────────────────────────────────────────────────────
B0    = 0.300      # ½ jarak roda penggerak (wheel_yoff)
D0    = 0.0        # CG traktor → poros roda penggerak (wheel_xoff = 0)
R0    = 0.079      # radius roda penggerak
L0    = 0.586      # poros roda → pin hitch (= base_length/2)
M_T0  = 113.485    # massa traktor (base+monitor+2 wheel+4 caster)
I0    = 15.40      # inersia yaw traktor (_I_Z)
IW2_0 = (3.610/12.0)*(3*0.079**2 + 0.055**2)   # roda traktor inersia thd z ≈ 0.00654
IWY_0 = 3.610*0.079**2/2.0                       # roda traktor inersia spin ≈ 0.01127

# ── TRAILER (trolley nested) ──────────────────────────────────────────────────
B     = 0.455      # ½ jarak roda trailer (tr_wheel_yoff)
D     = 0.495      # CG trailer → poros roda trailer (tr_wheel_xoff)
R     = 0.0725     # radius roda trailer
L     = 0.650      # CG trailer → pin hitch
M_T   = 12.0       # massa trailer (tr_base 5 + tr_pallet 5 + 2×0.5 + 2×0.5)
IW2   = (0.5/12.0)*(3*0.0725**2 + 0.055**2)      # roda trailer inersia thd z
IWY   = 0.5*0.0725**2/2.0                          # roda trailer inersia spin

# Inersia yaw trailer (box base + pallet + kontribusi roda/caster)
_I_base   = (5.0/12.0)*(1.000**2 + 1.300**2)       # 1.121
_I_pallet = (5.0/12.0)*(1.000**2 + 1.300**2)       # 1.121
_I_wheels = 2*0.5*(0.455**2 + 0.495**2)            # ~0.452
_I_cast   = 2*0.5*(0.400**2 + 0.555**2)            # ~0.468
I_TR  = _I_base + _I_pallet + _I_wheels + _I_cast   # ≈ 3.16

# ── Turunan ───────────────────────────────────────────────────────────────────
D_PRIME = L + D     # 1.145 — pin hitch → titik kontrol P trailer

# Parameter controller (Tabel 3 jurnal — koefisien kontrol, dipertahankan)
GAMMA = (1.1, 3.1, 3.2)     # γ1, γ2, γ3
Q_SMC = (0.7, 0.7, 0.6)     # Q1, Q2, Q3
P_UNC = (0.19, 2.4)         # P1, P2
# v_r diskalakan utk DYNAMIC SIMILARITY dgn jurnal: robot/oval kita ~4.3× lebih
# besar → v_r = 4.3 × 0.2 ≈ 0.85 m/s agar gain tak-berdimensi (ρd'/v²) = jurnal.
V_REF = 0.85                # kecepatan referensi trailer terskala (m/s)


if __name__ == '__main__':
    print('Parameter robot Polebot (mapping jurnal):')
    for k, v in sorted(globals().items()):
        if k.isupper() and isinstance(v, (int, float)):
            print(f'  {k:8s} = {v:.5f}')
    print(f"\n  d' = L + d = {D_PRIME:.3f} m")
