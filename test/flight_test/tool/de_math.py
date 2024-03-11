import sympy as sp
# 定义符号变量和常数
t, feq, t_offset, scale, c = sp.symbols('t feq t_offset scale c')

# 定义 PyTorch 代码对应的符号表达式
sin_t = sp.sin(t/feq + t_offset)
cos_t = sp.cos(t/feq + t_offset)
sin2p1 = sin_t**2 + 1/scale

x = cos_t / sin2p1
y = (sin_t * cos_t) / sin2p1
z = (c * sin_t) / sin2p1



# # 将符号表达式输出为 LaTeX 以便查看
# print("x(t):", sp.latex(x))
# print("y(t):", sp.latex(y))
# print("z(t):", sp.latex(z))

# 对 x, y, z 分别进行 t 的微分
dx_dt = sp.diff(x, t)
dy_dt = sp.diff(y, t)
dz_dt = sp.diff(z, t)


print("dx/dt:", dx_dt)
print("dy/dt:", dy_dt)
print("dz/dt:", dz_dt)
# # 将微分结果输出为 LaTeX 以便查看
# print("dx/dt:", sp.latex(dx_dt))
# print("dy/dt:", sp.latex(dy_dt))
# print("dz/dt:", sp.latex(dz_dt))