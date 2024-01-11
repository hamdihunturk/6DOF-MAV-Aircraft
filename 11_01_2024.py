import numpy as np
import matplotlib.pyplot as plt

def atmosfer_modeli(hASL):
    """
    Belirli bir irtifada atmosferin sıcaklık, basınç ve yoğunluğunu hesaplar.

    :param hASL: Deniz seviyesine göre irtifa (metre)
    :return: Sıcaklık (K), basınç (Pa), yoğunluk (kg/m^3)
    """
    # Sabitler
    R = 287.05  # [J/kg-K], kuru hava için özgül gaz sabiti
    P0 = 1013.25  # [N/m^2], deniz seviyesinde standart basınç
    T0 = 288.15  # [K], deniz seviyesinde standart sıcaklık
    g = 9.80665  # [m/s^2], yerçekimi sabiti
    L = 6.5 / 1000 # [K/m], alt atmosferde sıcaklık azalma oranı
    rho0 = 1.225
    # Sıcaklık hesabı
    T = T0 - L * hASL

    # Basınç hesabı
    P = P0 * (1 - L * hASL / T0) ** (g / (R * L))

    # Yoğunluk hesabı
    rho = rho0 * (P / P0) * (T0 / T)

    return T, P, rho

# 500 metre irtifadaki sıcaklık, basınç ve yoğunluk
hASL = 500  # [m]
T, P, rho = atmosfer_modeli(hASL)

print(f"İrtifa: {hASL} m")
print(f"Sıcaklık: {T:.2f} K")
print(f"Basınç: {P:.2f} Pa")
print(f"Yoğunluk: {rho:.2f} kg/m^3")

# değişebilen paremetreler
δt = 0.5 # Yüzdesel motor gücü ifade eder. Orneğin 0.5 değeri motor %50 hızda çalışıyor anlamına gelir.
elevator_acisi = 0 # elevatör açısı 0.2 saniye için 2°
kanatcik_acisi = 2 # kanatçık açısı 0.2 saniye için 2° 
dümen_acisi = 0


# Parametreler
Sprop = 0.2027    # m, Pervane çapı
Cprop = 1.0     # Pervane katsayısı
Ktp = 0       # İtme katsayısı 
K_ = 0       # Tork katsayısı 
k_motor = 80  # Motor sabiti
e = 0.9       # Motor verimliliği
#V = 50  # Hava aracının hızı (m/s)
RHO = rho  # Deniz seviyesinde hava yoğunluğu (kg/m^3)
S = 0.55  # Kanat alanı (m^2)
B = 2.8956  # Kanat açıklığı (m)
C = 0.18994   # Ortalama aerodinamik kord (m)



# δa Control signal denoting the aileron deflection (chapter 4)
# δe Control signal denoting the elevator deflection (chapter 4)
# δr Control signal denoting the rudder deflection (chapter 4)
# δt Control signal denoting the throttle deflection (chapter 4)
# δ değerleri
δe = 0
δa = 0 
δr = 0


motor_itme, motor_tork = 1,1

def hareket_denklemleri(state, t, kuvvetler_momentler, hava_araci_parametreleri):
    """
    # Sabit kanatlı bir hava aracının 6DOF hareket denklemleri.

    state: [u, v, w, p, q, r, phi, theta, psi, x, y, z] anlık durum vektörü
    Zaman (kullanılmıyor, entegrasyon uyumluluğu için)
    kuvvetler_momentler: [X, Y, Z, L, M, N] kuvvetler ve momentler
    hava_araci_parametreleri: Hava aracının parametreleri (kütle, atalet momentleri vb.)
    return: Zamanın türevine göre durum vektörünün değişim hızları

    """

    # Hava aracının durumu
    u, v, w, p, q, r, phi, theta, psi, x, y, z = state

    # Hava aracının parametreleri
    m = hava_araci_parametreleri['mass']
    Jxx =  hava_araci_parametreleri['Jxx'] # x ekseni etrafındaki atalet momenti
    Jyy =  hava_araci_parametreleri['Jyy'] # y ekseni etrafındaki atalet momenti
    Jzz =  hava_araci_parametreleri['Jzz'] # z ekseni etrafındaki atalet momenti
    Jxz =  hava_araci_parametreleri['Jxz'] # x ve z ekseni arasındaki çapraz atalet momenti
    
    # Γ teriminin hesaplanması
    Gamma = Jxx * Jzz - Jxz**2

    # Γ katsayılarının hesaplanması
    Gamma1 = Jxz * (Jxx - Jyy + Jzz) / Gamma
    Gamma2 = (Jzz * (Jzz - Jyy) + Jxz**2) / Gamma
    Gamma3 = Jzz / Gamma
    Gamma4 = Jxz / Gamma
    Gamma5 = (Jzz - Jxx) / Jyy
    Gamma6 = Jxz / Jyy
    Gamma7 = ((Jxx - Jyy) * Jxx + Jxz**2) / Gamma
    Gamma8 = Jxx / Gamma


    # Kuvvetler ve momentler
    Fx, Fz, Fy, L, M, N = kuvvetler_momentler
    
    # Rotasyonel hareket denklemleri
    dot_p = (Gamma1 * p * q - Gamma2 * q * r) + (Gamma3 * L + Gamma4 * N)
    dot_q = (Gamma5 * p * r - Gamma6 * (p**2 - r**2)) + (M / Jyy)
    dot_r = (Gamma7 * p * q - Gamma1 * q * r) + (Gamma4 * L + Gamma8 * N)
    print(p)
    # Translasyonel hareket denklemleri
    dot_u = r* v - q* w - Fx/m  
    dot_v = p*w - r*u + Fy/m
    dot_w = q*u - p*v + Fz/m
    
    # Euler açılarının türevleri
    dot_phi = p + q*np.sin(phi)*np.tan(theta) + r*np.cos(phi)*np.tan(theta)
    dot_theta = q*np.cos(phi) - r*np.sin(phi)
    dot_psi = q*np.sin(phi)/np.cos(theta) + r*np.cos(phi)/np.cos(theta)
    
    # Konumun türevleri
    s_phi, c_phi = np.sin(phi), np.cos(phi)
    s_theta, c_theta = np.sin(theta), np.cos(theta)
    s_psi, c_psi = np.sin(psi), np.cos(psi)
    dot_x = (c_theta * c_psi) * u + (s_phi * s_theta * c_psi - c_phi * s_psi) * v + (c_phi * s_theta * c_psi + s_phi * s_psi) * w
    dot_y = (c_theta * s_psi) * u + (s_phi * s_theta * s_psi + c_phi * c_psi) * v + (c_phi * s_theta * s_psi - s_phi * c_psi) * w
    dot_z = (-s_theta) * u + (s_phi * c_theta) * v + (c_phi * c_theta) * w


    return [dot_u, dot_v, dot_w, dot_p, dot_q, dot_r, dot_phi, dot_theta, dot_psi, dot_x, dot_y, dot_z]


def aerodinamik_kuvvetler_momentler(V, rho, S, b, c, p, q, r, phi, theta, alpha , betha,δe,δa):
    """
    # Aerodinamik kuvvetleri ve momentleri hesaplanan metod.

    V: Hava aracının hızı (m/s)
    rho: Hava yoğunluğu (kg/m^3)
    S: Kanat alanı (m^2)
    b: Kanat açıklığı (m)
    c: Ortalama aerodinamik kord (m)
    CL: Kaldırma kuvveti katsayısı
    CD: Sürükleme kuvveti katsayısı
    CY: Yan kuvvet katsayısı
    Cl: Dönme momenti katsayısı - Aileron Açısı - phi 
    Cm: Düşey moment katsayısı - Elevator Açısi - theta 
    Cn: Yanal moment katsayısı - Dümen Açısı - psi
    :return: Fx Fy,Fz, dönme momenti, düşey moment, yanal moment
    """

    # Dinamik basınç
    Pq = 0.5 * rho * V**2

    g = 9.81
    m = 13.5
    
    # CL sabitleri
    CLq = 0
    CLδe = -0.36
    CL0= 0.28
    CLa = 3.45
    CL_a = CL0 + CLa * alpha
    CL = CL_a + (CLq*c*q/ 2*V) + CLδe * δe
    
    # CD sabitleri
    CDq = 0
    CDδe = 0
    CD0= 0.03
    CDa = 0.30
    CD_a = CD0 + CDa * alpha
    CD = CD_a + (CDq*c*q/ 2*V) + CDδe * δe

    # CY sabitleri
    CY0 = 0
    CYb = -0.98
    CYp = 0
    CYr = 0 
    CYδa = 0
    CYδr = -0.17
    CY = CY0 + CYb * betha + CYp * b * p / 2 * V + CYr * b * r / 2 * V + CYδa * δa + CYδr * δr

    # Aerodinamik kuvvetler
    L = Pq * S * CL  # Kaldırma kuvveti
    D = Pq * S * CD  # Sürükleme kuvveti
    Y = Pq * S * CY  # Yan kuvvet

    
    Cx_a = -CD_a * np.cos(alpha) + CL_a * np.sin(alpha)
    Cxq_a = -CDq * np.cos(alpha) + CLq * np.sin(alpha)
    Cxδe_a = -CDδe * np.cos(alpha) + CLδe * np.sin(alpha)

    Cz_a = -CD_a * np.sin(alpha) - CL_a * np.cos(alpha)
    Czq_a  = -CDq * np.sin(alpha) - CLq * np.cos(alpha)
    Czδe_a = -CDδe * np.sin(alpha) - CLδe * np.cos(alpha)
    

    # Haraket denklemleri için  Fx,Fy,Fz
    Fx = -g * np.sin(theta)  + Pq * S * (Cx_a + Cxq_a * c * q/ 2 * V + Cxδe_a * δe) + motor_itme # Kaldırma kuvveti
    Fy = m * g *  np.cos(theta) * np.sin(phi) + Y + 0   # Sürükleme kuvveti 
    Fz = m * g *  np.cos(theta) * np.cos(phi) + Pq * S * (  Cz_a + Czq_a * c * q / 2 * Czδe_a *  δe) + 0 # Yan kuvvet

    # Cl sabitleri 
    Cl0 = 0
    Clb = -0.12
    Clp = -0.26
    Clr = 0.14
    Clδa = 0.08
    Clδr = 0.105

    # Cm sabitleri 
    Cm0 = -0.02338
    Cma = -0.38
    Cmq = -3.6
    Cmδe = -0.5

    # Cn sabitleri 
    Cn0 = 0
    Cnb = 0.25
    Cnp = 0.022
    Cnr = -0.35
    Cnδa = 0.06
    Cnδr = -0.032

    Cl = Cl0 + Clb * betha + Clp * b * p / 2 * V + Clr * b * r / 2 * V+ Clδa * δa + Clδr + δr
    Cm = Cm0 + Cma * alpha + Cmq * c * q / 2 * V + Cmδe * δe
    Cn = Cn0 + Cnb * betha + Cnp * b * p / 2 * V + Cnr * b * r / 2 * V+ Cnδa * δa + Cnδr + δr

    # Aerodinamik momentler
    l = Pq * S * b * Cl + motor_tork # Dönme momenti
    m = Pq * S * c * Cm  + 0 # Düşey moment
    n = Pq * S * b * Cn  + 0 # Yanal moment
    
    return Fx, Fy, Fz, l, m, n


class MotorModeli:
    def __init__(self,rho,Sprop,Cprop, k_motor,Ktp,K_,e):
        self.rho = rho       # Hava yoğunluğu
        self.Sprop = Sprop   # Pervane çapı
        self.Cprop = Cprop   # Pervane katsayısı
        self.Ktp = Ktp       # Tork sabiti
        self.K_ = K_         # 
        self.k_motor = k_motor  # Motor sabiti
        self.e = e          # Motor verimliliği

    def itme_tork_hesapla(self, V,δt):
        # İtme (FX_p) ve tork (Tp) hesaplamaları
        FX_p = 0.5 * self.rho * self.Sprop * self.Cprop * ( (self.k_motor * δt )**2 - V**2 )
        Tp = -self.Ktp * (self.K_ * δt)**2
        
        FX_p *= self.e
        Tp *= self.e
        return FX_p, Tp


def alpha_betha_hesapla(w,u,Va):
    """
    α (Alfa): Hücum açısı
    β (Beta): Yanal kayma açısı
    return: alpha , betha
    """

    alpha = np.arctan(w/u)
    betha = np.arcsin(u/Va)

    return alpha, betha 


def elevatör_etkisi(t):
    δe,δa = 0,0
    if 10.0 <= t <= 10.2:
        # iki tarafında aynı yönde olduğunu varsaydık
        δer = elevator_acisi 
        δel = elevator_acisi
        δe = δer + δel
        δa = -δer + δel
        return δe,δa
    else:
        return δe,δa
    
def kanatcik_etkisi(t):
    δe,δa = 0,0
    if 10.0 <= t <= 10.2:
        # iki tarafında aynı yönde olmadığını varsaydık
        δer = kanatcik_acisi 
        δel = -kanatcik_acisi
        δe = δer + δel
        δa = -δer + δel
        return δe,δa
    else:
        return δe,δa
    


# Simülasyon parametreleri
simulasyon_suresi = 30  # saniye
dt = 0.1  # Zaman adımı (saniye)
adim_sayisi = int(simulasyon_suresi / dt)

# Başlangıç durumu ve hava aracı parametreleri
initial_state = [0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 500]  # 0 hız ve 500 yükseklik kabul edip başlatalım
hava_araci_parametreleri = {'mass': 13.5, 'Jxx': 0.8244, 'Jyy': 1.135, 'Jzz': 1.759,'Jxz':0.1204}

# Sonuçları saklamak için listeler
zaman = np.linspace(0, simulasyon_suresi, adim_sayisi)
irtifa = []
hiz = []
dikilme_acisi = []
hucum_acisi = []
motor_itme_list = []
motor_tork_list = []
yatis_acisi = []
yönelme_acisi = []
alfa = 0.4712  # başlangıç alfa açısı
beta = 0.01 # başlangıç beta açısı




# Simülasyon döngüsü
for i in range(adim_sayisi):
    
    t = dt * i

    if( hiz != [] ):
        Va = hiz[i-1]
    else:
        Va = 50 # başangıç hız 50

    motor = MotorModeli(rho,Sprop,Cprop,k_motor,Ktp,K_,e)
    motor_itme, motor_tork, = motor.itme_tork_hesapla(Va,δt)

    #δe,δa = elevatör_etkisi(t)
    δe,δa = kanatcik_etkisi(t)

    alfa , beta = alpha_betha_hesapla(initial_state[2], initial_state[0],Va)   
    Fx, Fz, Fy, L, M, N= aerodinamik_kuvvetler_momentler(Va, rho, S, B, C, initial_state[4],initial_state[5],initial_state[6],initial_state[7],initial_state[8],alfa,beta,δe,δa)
    kuvvetler_momentler = [Fx, Fz, Fy, L, M, N]


    state_derivative = hareket_denklemleri(initial_state, t, kuvvetler_momentler, hava_araci_parametreleri)

    # birim zamanda değişiklikleri kaydet
    initial_state = [initial_state[j] + state_derivative[j]*0.002 for j in range(len(state_derivative))]
                                                          # dt
    
    # Sonuçları kaydet
    irtifa.append(initial_state[11])
    hiz.append(np.sqrt(initial_state[0]**2 + initial_state[1]**2 + initial_state[2]**2))
    dikilme_acisi.append(initial_state[7])  # theta
    yatis_acisi.append(initial_state[6]) # phi
    yönelme_acisi.append(initial_state[8]) # psi
    hucum_acisi.append(np.arctan2(initial_state[2], initial_state[0]))  # atan(w/u)
    motor_itme_list.append(motor_itme)
    motor_tork_list.append(motor_tork)

# Grafikleri çiz
plt.figure(figsize=(12, 8))

plt.subplot(3, 3, 1)
plt.plot(zaman, irtifa, label='İrtifa (m)')
plt.xlabel('Zaman (s)')
plt.ylabel('İrtifa (m)')
plt.title('İrtifa Zaman Grafiği')
plt.legend()

plt.subplot(3,3, 2)
plt.plot(zaman, hiz, label='Hız (m/s)')
plt.xlabel('Zaman (s)')
plt.ylabel('Hız (m/s)')
plt.title('Hız Zaman Grafiği')
plt.legend()


plt.subplot(3, 3, 3)
plt.plot(zaman, dikilme_acisi, label='Dikilme Açısı (derece)')
plt.xlabel('Zaman (s)')
plt.ylabel('Dikilme Açısı (derece)')
plt.title('Dikilme Açısı Zaman Grafiği')
plt.legend()

plt.subplot(3, 3, 4)
plt.plot(zaman, hucum_acisi, label='Hücum Açısı (derece)')
plt.xlabel('Zaman (s)')
plt.ylabel('Hücum Açısı (derece)')
plt.title('Hücum Açısı Zaman Grafiği')
plt.legend()


plt.subplot(3, 3, 5)
plt.plot(zaman, motor_itme_list , label='Motor İtme')
plt.xlabel('Zaman (s)')
plt.ylabel('')
plt.title('Motor İtme zaman grafiği')
plt.legend()


plt.subplot(3, 3, 6)
plt.plot(zaman,  motor_tork_list, label='Motor Tork')
plt.xlabel('Zaman (s)')
plt.ylabel('')
plt.title('Motor Tork zaman grafiği')
plt.legend()

plt.subplot(3, 3, 7)
plt.plot(zaman,  yatis_acisi, label='Yatış Açısı')
plt.xlabel('Zaman (s)')
plt.ylabel('')
plt.title('Yatış Açısı zaman grafiği')
plt.legend()

plt.subplot(3, 3, 8)
plt.plot(zaman,  yönelme_acisi, label='Yönelme Açısı')
plt.xlabel('Zaman (s)')
plt.ylabel('')
plt.title('Yönelme Açısı zaman grafiği')
plt.legend()


plt.tight_layout()
plt.show()
