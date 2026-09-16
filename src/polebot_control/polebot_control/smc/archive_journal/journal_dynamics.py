"""Dinamika tereduksi tractor-trailer (Alipour 2019) — H(q̄) & C̃(q̄,u) eksak.

Diturunkan secara simbolik (Lagrangian) SEKALI saat import, lalu di-lambdify
menjadi fungsi numerik cepat untuk controller:
    H(theta, theta0)            -> 2x2  (Eq.53: H = (ÑᵀB̃)⁻¹ ÑᵀM̃Ñ)
    Cterm(theta, theta0, v, w0) -> 2x1  (Eq.53: C̃ = (ÑᵀB̃)⁻¹ Ñᵀ(M̃Ṅu + Ṽ))

Validasi: dgn parameter jurnal, H ≈ [[1.62,0.13],[1.62,-0.13]] (Eq.94 jurnal).
Parameter robot kita diambil dari journal_params.
"""
import numpy as np
import sympy as sp

try:
    from . import journal_params as P
except ImportError:
    import journal_params as P

# ── Simbol ────────────────────────────────────────────────────────────────────
th, th0 = sp.symbols('theta theta0', real=True)
Xpd, Ypd, thd_f, th0d_f = sp.symbols('Xpd Ypd thd th0d', real=True)   # q̄̇ bebas
v, w0 = sp.symbols('v omega0', real=True)

# Parameter numerik (substitusi langsung — model robot kita)
_subs = {
    'Mt0': P.M_T0, 'I0': P.I0, 'mw0': 3.610, 'iwy0': P.IWY_0, 'b0': P.B0,
    'r0': P.R0, 'Lc0': P.L0 + P.D0,
    'Mt': P.M_T, 'Itr': P.I_TR, 'mw': 0.5, 'iwy': P.IWY, 'b': P.B,
    'r': P.R, 'd': P.D, 'dprime': P.D_PRIME,
}
sym = {k: sp.Float(val) for k, val in _subs.items()}
Mt0, I0, mw0, iwy0, b0, r0, Lc0 = (sym['Mt0'], sym['I0'], sym['mw0'], sym['iwy0'],
                                   sym['b0'], sym['r0'], sym['Lc0'])
Mt, Itr, mw, iwy, b, r, d, dp = (sym['Mt'], sym['Itr'], sym['mw'], sym['iwy'],
                                 sym['b'], sym['r'], sym['d'], sym['dprime'])

# ── Matriks massa BEBAS M̃(q̄) 4×4 — dari energi kinetik dgn q̄̇ bebas ──────────
qd = sp.Matrix([Xpd, Ypd, thd_f, th0d_f])
# kecepatan CG trailer (P + d sepanjang θ)
xtrd = Xpd - d*sp.sin(th)*thd_f
ytrd = Ypd + d*sp.cos(th)*thd_f
# kecepatan CG traktor (P + d'·θ + Lc0·θ0)
xtr0d = Xpd - dp*sp.sin(th)*thd_f - Lc0*sp.sin(th0)*th0d_f
ytr0d = Ypd + dp*sp.cos(th)*thd_f + Lc0*sp.cos(th0)*th0d_f
v0  = xtr0d*sp.cos(th0) + ytr0d*sp.sin(th0)     # kecepatan maju traktor
vtr = Xpd*sp.cos(th) + Ypd*sp.sin(th)           # kecepatan maju trailer di P

T  = sp.Rational(1,2)*Mt *(xtrd**2 + ytrd**2)  + sp.Rational(1,2)*Itr*thd_f**2
T += sp.Rational(1,2)*Mt0*(xtr0d**2 + ytr0d**2) + sp.Rational(1,2)*I0 *th0d_f**2
T += sp.Rational(1,2)*iwy0*(((v0 + b0*th0d_f)/r0)**2 + ((v0 - b0*th0d_f)/r0)**2)
T += sp.Rational(1,2)*mw0*(2*v0**2 + 2*(b0*th0d_f)**2)
T += sp.Rational(1,2)*iwy *(((vtr + b*thd_f)/r)**2 + ((vtr - b*thd_f)/r)**2)
T += sp.Rational(1,2)*mw *(2*vtr**2 + 2*(b*thd_f)**2)

Mtil = sp.Matrix(sp.hessian(T, qd))             # 4×4, fungsi (θ,θ0)

# ── Null space Ñ(q̄) 4×2 (Eq.48) ──────────────────────────────────────────────
Ntil = sp.Matrix([[sp.cos(th),                  0],
                  [sp.sin(th),                  0],
                  [sp.tan(th0 - th)/dp,         0],
                  [0,                           1]])

# ── Input map W (u → spin roda traktor); H = (Wᵀ)⁻¹ ÑᵀM̃Ñ ──────────────────────
v0_u = (sp.cos(th)*sp.cos(th0) + sp.sin(th)*sp.sin(th0))*v \
       - Lc0*sp.sin(th0)*sp.cos(th0)*w0 + Lc0*sp.cos(th0)*sp.sin(th0)*w0  # = v·cos(θ0-θ)
v0_u = v*sp.cos(th0 - th)                         # kecepatan maju traktor dlm u
phi1d = (v0_u - b0*w0)/r0
phi2d = (v0_u + b0*w0)/r0
W = sp.Matrix([[sp.diff(phi1d, v), sp.diff(phi1d, w0)],
               [sp.diff(phi2d, v), sp.diff(phi2d, w0)]])

# ── Lambdify komponen (cepat di runtime) ──────────────────────────────────────
_f_Mtil = sp.lambdify((th, th0), Mtil, 'numpy')
_f_Ntil = sp.lambdify((th, th0), Ntil, 'numpy')
_f_W    = sp.lambdify((th, th0), W, 'numpy')
# ∂Ñ/∂θ dan ∂Ñ/∂θ0 untuk Ṅ
_f_dN_th  = sp.lambdify((th, th0), sp.diff(Ntil, th),  'numpy')
_f_dN_th0 = sp.lambdify((th, th0), sp.diff(Ntil, th0), 'numpy')
# ∂M̃/∂θ dan ∂M̃/∂θ0 untuk suku Coriolis Ṽ (Christoffel)
_f_dM_th  = sp.lambdify((th, th0), sp.diff(Mtil, th),  'numpy')
_f_dM_th0 = sp.lambdify((th, th0), sp.diff(Mtil, th0), 'numpy')


def H(theta, theta0):
    """Matriks H 2×2: τ = H·[v̇, ω̇0]ᵀ + C̃."""
    Mt_ = np.array(_f_Mtil(theta, theta0), dtype=float)
    Nt_ = np.array(_f_Ntil(theta, theta0), dtype=float)
    W_  = np.array(_f_W(theta, theta0), dtype=float)
    Hkin = Nt_.T @ Mt_ @ Nt_                       # ÑᵀM̃Ñ (= H_kin 2×2)
    return np.linalg.inv(W_.T) @ Hkin


def Cterm(theta, theta0, v_, w0_):
    """Vektor Coriolis/sentripetal C̃ 2×1 (Eq.53b)."""
    Nt_  = np.array(_f_Ntil(theta, theta0), dtype=float)
    W_   = np.array(_f_W(theta, theta0), dtype=float)
    Mt_  = np.array(_f_Mtil(theta, theta0), dtype=float)
    u    = np.array([v_, w0_], dtype=float)
    qdot = Nt_ @ u                                  # q̄̇ = Ñu  (4×1)
    # Ṅ = (∂Ñ/∂θ)·θ̇ + (∂Ñ/∂θ0)·θ̇0
    Ndot = (np.array(_f_dN_th(theta, theta0), dtype=float)  * qdot[2]
            + np.array(_f_dN_th0(theta, theta0), dtype=float) * qdot[3])
    # Ṽ (Coriolis dari M̃, Christoffel): Ṽ_k = Σ (∂M̃_ki/∂q_j − ½∂M̃_ij/∂q_k) q̇_i q̇_j
    dM = [np.zeros((4,4)), np.zeros((4,4)),
          np.array(_f_dM_th(theta, theta0), dtype=float),
          np.array(_f_dM_th0(theta, theta0), dtype=float)]   # ∂M̃/∂[Xp,Yp,θ,θ0]
    Vtil = np.zeros(4)
    for k in range(4):
        s = 0.0
        for i in range(4):
            for j in range(4):
                c = dM[j][k, i] - 0.5*dM[k][i, j]
                s += c * qdot[i] * qdot[j]
        Vtil[k] = s
    rhs = Mt_ @ Ndot @ u + Vtil                     # M̃Ṅu + Ṽ  (4×1)
    return np.linalg.inv(W_.T) @ (Nt_.T @ rhs)


# ── Validasi parameter jurnal saat dijalankan langsung ────────────────────────
if __name__ == '__main__':
    print('H robot kita (θ=θ0=0):'); print(np.round(H(0.0, 0.0), 4))
    print('H robot kita (menikung θ0-θ=20°):'); print(np.round(H(0.0, np.pi/9), 4))
    print('C̃ (θ0-θ=20°, v=0.2, ω0=0.1):'); print(np.round(Cterm(0.0, np.pi/9, 0.2, 0.1), 4))
    print('C̃ (lurus, v=0.2, ω0=0):'); print(np.round(Cterm(0.0, 0.0, 0.2, 0.0), 4))
