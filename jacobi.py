import sympy as sp

sp.init_printing(use_unicode=True)

a, b, c, d = sp.symbols('a b c d')          #symbole definieren

ox, oy, oz, dt = sp.symbols('ox oy oz dt')  #symbole definieren

F = sp.Matrix([                             #schiefsymmetrische matrix aus hamilton produkt aus winkelgeschw.
            [0,-ox, -oy, -oz],
            [ox,0, oz, -oy],
            [oy, -oz, 0, ox,],
            [oz, oy, -ox, 0]

])
x = sp.Matrix([                             # zustandsvektor aus quaternionen
    [a],
    [b],
    [c],
    [d]
])

neuer_zustand = x + 0.5 * F * x * dt        #iterativelösung des neuen quaternion

jabo_n_zustand = neuer_zustand.jacobian(x)  #jacobimatrix berechnung nach partialableitung nach quaternion

print("Aj =")
#sp.pprint(jabo_n_zustand)
######################################################33

H = sp.Matrix([                             #prädikator der beschleunigung
    [2*(b*d - a*c)],
    [2*(c*d + a*b)],
    [a**2-b**2-c**2+d**2]
])

H_jacob = H.jacobian(x)                     #jacobimatrix, partielableitung nach qauternionen

print("Cj =")
#sp.pprint(H_jacob)

#######################################
z = sp.Matrix([                            
    [ox],
    [oy],
    [oz]
])

Gd = (F * x).jacobian(z)
#sp.pprint(Gd)

###########################################
#umrechnung von quaternion in euler

def quattoeul():
    a, b, c, d = sp.symbols('a b c d', real = True) 


    c_11 = a**2 + b**2 - c**2 - d**2
    c_21 = 2 * (b * c - a * d)
    c_31 = 2 * (b * d - a * c)
    c_32 = 2 * (c * d+ a * b)
    c_33 = a**2 - b**2 -c**2 + d**2


    phi = sp.atan2(c_32,c_33)
    theta = sp.asin(-c_31)
    psi = sp.atan2(c_21,c_11)

    
    return phi, theta, psi

ergebnis_phi, ergebnis_theta, ergebnis_psi = quattoeul()



print("Roll (Phi):")
print(ergebnis_phi)
print("\nPitch (Theta):")
print(ergebnis_theta)
print("\nYaw (Psi):")
print(ergebnis_psi)




