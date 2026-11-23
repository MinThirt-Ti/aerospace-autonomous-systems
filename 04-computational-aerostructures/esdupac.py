import sys
import math

def fortran_e(val, width, dec):
    if val == 0.0:
        return f" 0.{'0'*dec}E+00".rjust(width)
    v = abs(val)
    exp = int(math.floor(math.log10(v))) + 1
    man = v / (10**exp)
    man_rounded = round(man, dec)
    if man_rounded >= 1.0:
        man_rounded /= 10.0
        exp += 1
    elif man_rounded < 0.1 and man_rounded > 0:
        man_rounded *= 10.0
        exp -= 1
        
    man_str = f"{man_rounded:.{dec}f}"[2:]
    sign = "-" if val < 0 else " "
    res = f"{sign}0.{man_str}E{exp:+03d}"
    return res.rjust(width)

# C *****************************************************************************
# C ESDU International plc.
# C
# C     Program header added                                      Amendment D
#       PROGRAM A8335
# C
# C     ESDUpac Number:   A8335V42  (Previously 1038)
# C     ESDUpac Title:    THE ESTIMATION OF THE STIFFNESSES AND APPARENT
# C                       ELASTIC PROPERTIES OF LAMINATED FLAT PLATES.
# C
# C     Data Item Number: 833035
# C     Data Item Title:  ESTIMATION OF THE STIFFNESSES AND APPARENT
# C                       ELASTIC PROPERTIES OF LAMINATED FLAT PLATES
# C     ESDUpac Status
# C     --------------
# C     Version:   1.0    NOVEMBER 1983
# C     Version:   2.0    OCTOBER 1988 - Data Item Amendment A
# C                       Input changed to free format. Alpha and beta
# C                       replaced by x and y respectively in Subroutine
# C                       G83035.
# C     Version:   3.0    OCTOBER 1989 - Data Item Amendment B
# C                       Program restructured with file input fascility
# C                       added.
# C     Version:   4.0    DECEMBER 1989 - Data Item Amendment C.
# C                       No change to FORTRAN program. (Changes affected
# C                       BASIC program only.)
# C     Version:   4.1    AUGUST 1991 - Data Item Amendment D
# C                       Standard program and output headers added.
# C                       All variables explicitly declared. Output
# C                       reformatted for 80 column printer.
# C     Version:   4.2    NOVEMBER 1994 - Data Item Amendment E
# C                       Sign of B matrix changed (Subroutine AG8335).
# C                       Subroutine names changed according to current
# C                       standards. Facility to input run title and 
# C                       output file name added.
# C
# C .............................................................................
# C
# C     PURPOSE
# C         THE ESTIMATION OF THE STIFFNESSES AND APPARENT
# C         ELASTIC PROPERTIES OF LAMINATED FLAT PLATES.
# C
# C     REMARKS
# C         COMBINATIONS OF LAYERS OF ISOTROPIC AND ORTHOTROPIC
# C         MATERIALS MAY BE CONSIDERED.
# C
#         FOR COMPOSITE LAMINA,STIFFNESSES MAY BE INPUT OR
# C         ESTIMATED FROM THE CONSTITUENT FIBRE AND MATRIX
# C         PROPERTIES.
# C
# C         IF LAMINA PROPERTIES ARE TO BE ESTIMATED, THE PROGRAM
# C         LIMITS THE MAXIMUM NUMBER OF LAYERS FOR WHICH THE
# C         PROPERTIES ARE TO BE ESTIMATED TO 5. TO INCREASE THAT
# C         LIMIT THE DIMENSIONS FOR ARRAYS Q1 AND Q2 MUST BE
# C         MODIFIED ACCORDINGLY. SIMILARLY, THE NUMBER OF LAYERS IN
# C         THE PLATE IS LIMITED TO 50. TO INCREASE THAT LIMIT THE
# C         DIMENSION OF ARRAY Q MUST BE MODIFIED AS REQUIRED.
# C
# C         DATA MAY BE INPUT FROM THE KEYBOARD OR FROM A FILE CALLED
# C         'IN8335'. DATA ARE INPUT IN FREE FORMAT AND OUTPUT IS
# C         WRITTEN TO A FILE NAMED IN THE INPUT.                 Amendment E
# C
# C     METHOD
# C         PLATE STIFFNESSES ARE ESTIMATED USING THE METHOD IN
# C         ESDU ITEM NO. 94003. LAMINA ELASTIC PROPERTIES ARE
# C         FOUND USING THE LAW OF MIXTURES, THE HALPIN-TSAI
# C         EQUATIONS OR THE HASHIN-ROSEN EQUATIONS AS APPROPRIATE.
# C
# C-----------------------------------------------------------------------
# C
# C     Full definition of variables added                        Amendment E
# C     DEFINITION OF COMMOM VARIABLES
# C      A      - in-plane stiffness sub-matrix
# C      B      - coupled in-plane and flexural stiffness sub-matrix
# C      D      - flexural stiffness sub-matrix
# C      E      - array of plate apparent elastic properties
# C      H1     - plate thickness
# C      IN     - keyboard input channel
# C      IND    - input/output control parameter
# C      INFL   - file input channel
# C      INPUT  - indicator for file or keyboard input
# C      IPROP  - indicator for calculation of lamina properties
# C      ISYM   - plate symmetry code 
# C      N1     - number of layers in plate
# C      N2     - number of lamina for which lamina properties are to
# C               be estimated 
# C      N3     - number of layers for which data are input
# C      OUT1   - screen input channel
# C      OUT2   - file output channel 
# C      OUTFN  - output file name
# C      Q      - array of plate properties
# C      Q1     - array of fibre and matrix properties
# C      Q2     - array of estimated lamina properties
# C      TITLE1 - line 1 of title
# C      TITLE2 - line 2 of title
# C      TITLE3 - line 3 of title
# C      X9     - indicator for special orthotropy 
# C
# C     SUBROUTINES REQUIRED
# C       AA8335 
# C       AB8335
# C       AC8335
# C       AE8335
# C       AF8335
# C       AG8335
# C       AI8335
# C       AJ8335
# C       AK8335
# C
# C     THIS SEGMENT IS THE CONTROLLING ROUTINE
def A8335(filename):
    # Simulated COMMON BLOCK state
    S = {'N1':0, 'N2':0, 'N3':0, 'IND':1, 'IPROP':1, 'ISYM':0, 'LIDX':0, 
         'H1':0.0, 'X9':0.0, 'E1':0.0, 'E2':0.0, 'E3':0.0, 'E4':0.0, 'E5':0.0,
         'TITLE1':'', 'TITLE2':'', 'TITLE3':'', 'OUTFN':''}
         
    Q = [[0.0]*8 for _ in range(50)]
    Q1 = [[0.0]*10 for _ in range(5)]
    Q2 = [[0.0]*4 for _ in range(5)]
    A = [[0.0]*3 for _ in range(3)]
    B = [[0.0]*3 for _ in range(3)]
    D = [[0.0]*3 for _ in range(3)]
    
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    # C     CALL SUBROUTINE TO READ IN VARIABLES
    while True:
        # C Following line changed                                        Amendment E
        #    30 CALL AB8335(Q,Q1,Q2,N1,N2,N3,IND,IPROP,ISYM,TITLE1,TITLE2,TITLE3,OUTFN)
        AB8335(Q, Q1, Q2, S, lines)
        
        #    40 IF(IND.GT.1) GO TO 50
        if S['IND'] > 1:
            break
            
        # C     CALL SUBROUTINE TO OUTPUT HEADER AND RUN TITLE
        AK8335(A, B, D, Q, Q1, Q2, S)
        
        # C     CALL SUBROUTINE TO SET PARAMETERS FOR READING LAYER PROPERTIES
        AC8335(Q, S)
        S['IND'] = 2
        
        if S['IPROP'] < 2:
            continue
            
        # C     CALL SUBROUTINE TO ESTIMATE ELASTIC PROPERTIES FROM GIVEN FIBRE
        # C     AND MATRIX PROPERTIES
        AE8335(Q1, Q2, S)

    # C     CALL SUBROUTINE TO SET SYMMETRIC AND ANTI-SYMMETRIC HALF PLATE
    # C     LAYER PROPERTIES AND EVALUATE LAYER Z-C00RDINATE
    #    50 CALL AF8335(Q,H1,N1,N3,ISYM)
    AF8335(Q, S)
    
    # C     CALL SUBROUTINE TO CALCULATE PLATE STIFFNESS MATRICES
    AG8335(A, B, D, Q, S)
    
    # C     CALL SUBROUTINE TO CHECK FOR SPECIAL ORTHOTROPY
    AI8335(A, B, D, S)
    
    # C     CALL SUBROUTINE TO EVALUATE THE PLATE APPARENT ELASTIC PROPERTIES
    AJ8335(A, S)
    
    # C     CALL SUBROUTINE TO OUTPUT DATA TO A FILE
    AK8335(A, B, D, Q, Q1, Q2, S)


# C-----------------------------------------------------------------------
# C
# C Following line changed                                        Amendment E
#       SUBROUTINE AB8335(Q,Q1,Q2,N1,N2,N3,IND,IPROP,ISYM,TITLE1,TITLE2,TI
#      1TLE3,OUTFN)
# C
# C     PURPOSE
# C       TO READ IN VARIABLES FROM A FILE CALLED 'IN8335'
# C
# C     LOCAL VARIABLES
# C       I  - layer property loop counter
# C       I1 - yes/no indicator
# C       IX - layer input code
# C       NI - calculated properties layer number
def AB8335(Q, Q1, Q2, S, lines):
    if S['IND'] == 2:
        pass # GO TO 30
    else:
        # C     READ IN TITLE AND OUTPUT FILE NAME
        S['TITLE1'] = lines[S['LIDX']]; S['LIDX'] += 1
        S['TITLE2'] = lines[S['LIDX']]; S['LIDX'] += 1
        S['TITLE3'] = lines[S['LIDX']]; S['LIDX'] += 1
        S['OUTFN'] = lines[S['LIDX']]; S['LIDX'] += 1
        
        # C     INPUT NUMBER OF LAYERS AND SYMMETRY CODE
        parts = lines[S['LIDX']].split(); S['LIDX'] += 1
        S['N1'] = int(parts[0])
        S['ISYM'] = int(parts[1])
        
        I1 = int(lines[S['LIDX']]); S['LIDX'] += 1
        if I1 < 1: S['IPROP'] = 1
        if I1 >= 1:
            # C     READ IN DATA FOR LAMINA PROPERTY ESTIMATION
            S['IPROP'] = 2
            S['N2'] = int(lines[S['LIDX']]); S['LIDX'] += 1
            for i in range(S['N2']):
                parts = lines[S['LIDX']].replace(',', ' ').split(); S['LIDX'] += 1
                for j in range(5): Q1[i][j] = float(parts[j])
                parts = lines[S['LIDX']].replace(',', ' ').split(); S['LIDX'] += 1
                for j in range(4): Q1[i][j+5] = float(parts[j])
        return # GO TO 70

    # C     READ IN LAYER PROPERTIES
    #    30 DO 60 I=1,N3
    for i in range(S['N3']):
        parts = lines[S['LIDX']].replace(',', ' ').split(); S['LIDX'] += 1
        IX = int(parts[0])
        Q[i][0] = float(parts[1])
        
        if IX == 0:
            parts = lines[S['LIDX']].replace(',', ' ').split(); S['LIDX'] += 1
            Q[i][1] = float(parts[0])
            Q[i][2] = float(parts[1])
            Q[i][3] = float(parts[2])
            Q[i][4] = float(parts[3])
            Q[i][5] = float(parts[4])
        elif IX > 0:
            # C     CALL SUBROUTINE TO SET KNOWN LAYER PROPERTIES
            AD8335(Q, Q2, i, 0, IX)
        else:
            parts = lines[S['LIDX']].replace(',', ' ').split(); S['LIDX'] += 1
            Q[i][1] = float(parts[0])
            NI = int(parts[1])
            AD8335(Q, Q2, i, NI, IX)


# C-----------------------------------------------------------------------
# C
#       SUBROUTINE AC8335(Q,N1,N3,ISYM)
# C
# C     PURPOSE
# C       TO SET PARAMETERS FOR READING LAYER PROPERTIES
# C
# C     LOCAL VARIABLES
# C       I,J - loop counters
# C       X1  - odd or even layer parameter
def AC8335(Q, S):
    # C     ZERO ELEMENTS OF LAYER PROPERTIES MATRIX
    for i in range(50):
        for j in range(8):
            Q[i][j] = 0.0
            
    # C     SET COUNTER FOR NUMBER OF LAYER PROPERTIES TO BE INPUT
    if S['ISYM'] == 3:
        S['N3'] = S['N1']
    else:
        X1 = S['N1'] / 2.0 - int(S['N1'] / 2.0)
        if X1 > 0.01:
            S['N3'] = int(S['N1'] / 2) + 1
        else:
            S['N3'] = int(S['N1'] / 2)


# C-----------------------------------------------------------------------
# C
#       SUBROUTINE AD8335(Q,Q2,I,NI,IX)
# C
# C     PURPOSE
# C       TO SET KNOWN VALUES IN LAYER PROPERTY MATRIX
# C
# C     LOCAL VARIABLE
# C       J - loop counter
def AD8335(Q, Q2, i, ni, ix):
    if ix > 0:
        for j in range(1, 6):
            Q[i][j] = Q[ix-1][j]
    else:
        Q[i][2] = Q2[ni-1][0]
        Q[i][3] = Q2[ni-1][1]
        Q[i][4] = Q2[ni-1][2]
        Q[i][5] = Q2[ni-1][3]


# C-----------------------------------------------------------------------
# C
#       SUBROUTINE AE8335(Q1,Q2,N2)
# C
# C     PURPOSE
# C       TO ESTIMATE LAMINA ELASTIC PROPERTIES FROM GIVEN FIBRE AND
# C       MATRIX PROPERTIES
# C
# C     LOCAL VARIABLES
# C       I        - loop counter
# C       X1 to X3 - calculation parameters
def AE8335(Q1, Q2, S):
    for i in range(S['N2']):
        Q1[i][9] = 1.0 - Q1[i][4]
        Q2[i][0] = Q1[i][0]*Q1[i][4] + Q1[i][5]*Q1[i][9]
        X1 = Q1[i][1] / Q1[i][6]
        X2 = (X1 - 1.0) / (X1 + 2.0) * Q1[i][4]
        Q2[i][1] = Q1[i][6] * (1.0 + 2.0*X2) / (1.0 - X2)
        
        X1 = Q1[i][2] / Q1[i][7]
        X2 = X1 * (1.0 + Q1[i][4]) + Q1[i][9]
        X3 = X1 * Q1[i][9] + 1.0 + Q1[i][4]
        Q2[i][2] = Q1[i][7] * X2 / X3
        Q2[i][3] = Q1[i][3]*Q1[i][4] + Q1[i][8]*Q1[i][9]


# C-----------------------------------------------------------------------
# C
#       SUBROUTINE AF8335(Q,H1,N1,N3,ISYM)
# C
# C     PURPOSE
# C       TO SET SYMMETRIC AND ANTI-SYMMETRIC HALF PLATE LAYER PROPERTIES
# C       AND EVALUATE LAYER Z-COORDINATE
# C
# C     LOCAL VARIABLES
# C       I,J - loop counters
# C       N   - difference between N1 and N3 
# C       X1  - calculation parameter 
def AF8335(Q, S):
    if S['ISYM'] != 3:
        # C     SET HALF-PLATE LAYER PROPERTIES
        X1 = -1.0 if S['ISYM'] == 2 else 1.0
        N = S['N1'] - S['N3']
        for i in range(N):
            Q[S['N1'] - 1 - i][0] = Q[i][0] * X1
            for j in range(1, 6):
                Q[S['N1'] - 1 - i][j] = Q[i][j]

    # C     EVALUATE PLATE THICKNESS AND FORM LAYER PARAMETER Q(I,8)
    S['H1'] = 0.0
    for i in range(S['N1']):
        S['H1'] += Q[i][1]
        Q[i][7] = 1.0 - Q[i][5]*Q[i][5]*Q[i][3] / Q[i][2]

    # C     EVALUATE LAYER Z-COORDINATE
    X1 = 0.0
    for i in range(S['N1']):
        Q[i][6] = S['H1']/2.0 - (X1 + Q[i][1]/2.0)
        X1 += Q[i][1]


# C.....................................................................
# C
#       SUBROUTINE AG8335(A,B,D,Q,N1)
# C
# C     PURPOSE 
# C       TO FORM THE PLATE A,B AND D STIFFNESSES MATRICES
# C
# C     LOCAL VARIABLES
# C       C        - matrix relating stresses and strains
# C       GAM      - conversion factor from degrees to radians
# C       I,IJ     - loop counters
# C       I,JJ     - loop counters 
# C       T        - matrix relating strains in plate coordinates to
# C                  strains in layer coordinates 
# C       T1       - transpose of T-matrix
# C       U,V      - intermediate matrices
# C       X1 to X4 - calculation parameters
def AG8335(A, B, D, Q, S):
    GAM = 57.2957795
    C = [[0.0]*3 for _ in range(3)]
    T = [[0.0]*3 for _ in range(3)]
    T1 = [[0.0]*3 for _ in range(3)]
    W = [[0.0]*3 for _ in range(3)]
    U = [[0.0]*3 for _ in range(3)]

    for i in range(3):
        for j in range(3):
            A[i][j] = 0.0
            B[i][j] = 0.0
            C[i][j] = 0.0
            D[i][j] = 0.0

    for i in range(S['N1']):
        C[0][0] = Q[i][2] / Q[i][7]
        C[0][1] = Q[i][5] * Q[i][3] / Q[i][7]
        C[1][0] = C[0][1]
        C[1][1] = Q[i][3] / Q[i][7]
        C[2][2] = Q[i][4]
        
        X1 = math.sin(Q[i][0] / GAM)
        X2 = math.cos(Q[i][0] / GAM)
        X3 = math.sin(2.0 * Q[i][0] / GAM)
        X4 = math.cos(2.0 * Q[i][0] / GAM)
        
        T[0][0] = X2 * X2
        T[0][1] = X1 * X1
        T[0][2] = 0.5 * X3
        T[1][0] = T[0][1]
        T[1][1] = T[0][0]
        T[1][2] = -T[0][2]
        T[2][0] = -X3
        T[2][1] = X3
        T[2][2] = X4
        
        for ij in range(3):
            for jj in range(3):
                T1[ij][jj] = T[jj][ij]
                
        AH8335(T1, C, W)
        AH8335(W, T, U)
        
        X1_var = Q[i][1]
        X2_var = X1_var * Q[i][6]
        X3_var = X2_var * Q[i][6] + (Q[i][1]**3) / 12.0
        
        for ij in range(3):
            for jj in range(3):
                A[ij][jj] += X1_var * U[ij][jj]
                # C Sign change in following line                                 Amendment E
                B[ij][jj] += X2_var * U[ij][jj]
                D[ij][jj] += X3_var * U[ij][jj]


# C.....................................................................
# C
#       SUBROUTINE AH8335(V,W,U)
# C
# C     PURPOSE
# C       TO PERFORM THE MATRIX MULTIPLICATION V*W
# C
# C     LOCAL VARIABLES
# C       I,J,K - loop counters
# C       U     - product of V times W
# C       V,W   - matrices to be multiplied 
def AH8335(V, W, U):
    for i in range(3):
        for j in range(3):
            U[i][j] = 0.0
            for k in range(3):
                U[i][j] += V[i][k] * W[k][j]


# C.....................................................................
# C
#       SUBROUTINE AI8335(A,B,D,X9)
# C
# C     PURPOSE
# C       TO INVESTIGATE THE PLATE STIFFNESS MATRICES TO CHECK FOR
# C       SPECIAL ORTHOTROPY
# C
# C     LOCAL VARIABLES
# C       A1    - maximum number in A-matrix
# C       A2    - sum of elements in A-matrix that are zero for
# C               special orthotropy
# C       B1    - scaling factor for B-matrix
# C       B2    - sum of elements in B-matrix 
# C       D1    - maximum number in D-matrix
# C       D2    - sum of elements in D-matrix that are zero for
# C               special orthotropy
# C       J1,J2 - loop counters 
# C       J12   - sum of J1 and J2
def AI8335(A, B, D, S):
    A1 = 0.0
    D1 = 0.0
    for j1 in range(3):
        for j2 in range(3):
            if abs(A[j1][j2]) > A1: A1 = abs(A[j1][j2])
            if abs(D[j1][j2]) > D1: D1 = abs(D[j1][j2])
            
    B1 = math.sqrt(A1 * D1)
    A2 = 0.0
    B2 = 0.0
    D2 = 0.0
    
    for j1 in range(3):
        for j2 in range(3):
            if B1 > 0 and abs(B[j1][j2]) / B1 > 1.0E-6:
                B2 += abs(B[j1][j2])
            if B1 > 0 and abs(B[j1][j2]) / B1 <= 1.0E-6:
                B[j1][j2] = 0.0
            elif B1 == 0:
                B[j1][j2] = 0.0
                
            J12 = j1 + j2
            if j1 == j2 or J12 == 1: # Equivalency: Fortran skips (1,1),(2,2),(3,3) and (1,2),(2,1)
                pass
            else:
                if A1 > 0 and abs(A[j1][j2]) / A1 > 1.0E-6:
                    A2 += abs(A[j1][j2])
                if A1 > 0 and abs(A[j1][j2]) / A1 <= 1.0E-6:
                    A[j1][j2] = 0.0
                elif A1 == 0:
                    A[j1][j2] = 0.0
                    
                if D1 > 0 and abs(D[j1][j2]) / D1 > 1.0E-6:
                    D2 += abs(D[j1][j2])
                if D1 > 0 and abs(D[j1][j2]) / D1 <= 1.0E-6:
                    D[j1][j2] = 0.0
                elif D1 == 0:
                    D[j1][j2] = 0.0
                    
    S['X9'] = A2 + B2 + D2


# C.....................................................................
# C
#       SUBROUTINE AJ8335(A,E,H1)
# C
# C     PURPOSE
# C       TO EVALUATE THE PLATE APPARENT ELASTIC PROPERTIES. THESE
# C       PROPERTIES ARE OBTAINED FROM THE A-MATRIX
# C
# C     LOCAL VARIABLES
# C       AA  - A-matrix with elements divided by plate thickness
# C       I,J - loop counter
# C       X1  - product of Poisson's ratios - 1.0
def AJ8335(A, S):
    AA = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            AA[i][j] = A[i][j] / S['H1']
            
    # C     EVALUATE POISSON'S RATIOS
    S['E4'] = AA[1][0] / AA[1][1]
    S['E5'] = AA[0][1] / AA[0][0]
    X1 = 1.0 - S['E4'] * S['E5']
    
    # C     EVALUATE YOUNG'S MODULI AND SHEAR MODULUS
    S['E1'] = AA[0][0] * X1
    S['E2'] = AA[1][1] * X1
    S['E3'] = AA[2][2]


# C-----------------------------------------------------------------------
# C
# C Following line changed                                        Amendment E
#       SUBROUTINE AK8335(A,B,D,Q,Q1,Q2,E,N1,N2,IPROP,ISYM,X9,H1,IND,TITLE
#      11,TITLE2,TITLE3,OUTFN)
# C
# C     PURPOSE
# C       TO OUTPUT DATA TO A FILE NAMED IN THE INPUT             Amendment E
# C
# C     LOCAL VARIABLES
# C       I,J,K - loop counters
def AK8335(A, B, D, Q, Q1, Q2, S):
    if S['IND'] <= 1:
        # C     Output header changed                                     Amendment D
        print(" ----------------------------------------------------------------------------")
        print(" ESDU International plc.")
        print(" PROGRAM A8335\n")
        print(" ESDUpac Number:   A8335V42")
        print(" ESDUpac Title:    Estimation of the stiffnesses and apparent")
        print("                   elastic properties of laminated flat plates")
        print(" Data Item Number: 83035")
        print(" Data Item Title:  Estimation of the stiffnesses and apparent")
        print("                   elastic properties of laminated flat plates")
        print(" ESDUpac Version:  4.2 Issued November 1994")
        print(" (See Data Item for full input/output specification and interpretation)")
        print(" ----------------------------------------------------------------------------")
        # C     Following five lines added                                Amendment E  
        print("\n    TITLE OF RUN :")
        print(f"    {S['TITLE1']}")
        print(f"    {S['TITLE2']}")
        print(f"    {S['TITLE3']}")
        print("    OUTPUT FILE NAME :")
        print(f"    {S['OUTFN']}\n")
        print("    ----------------------------------------------------------------------------")
        return
        
    if S['IPROP'] >= 2:
        # C     OUTPUT ESTIMATED LAMINA PROPERTIES
        for i in range(S['N2']):
            print(f"\n    LAMINA{i+1:2d} PROPERTIES\n")
            print("             E(ALPHA)     E(BETA)    G(ALPHA- POISSON'S RATIO   VOLUME")
            print("                                       BETA)   (ALPHA-BETA)    FRACTION")
            print(f"    FIBRE   {fortran_e(Q1[i][0], 10, 4)}  {fortran_e(Q1[i][1], 10, 4)}  {fortran_e(Q1[i][2], 10, 4)}{Q1[i][3]:8.3f}{Q1[i][4]:15.3f}")
            print(f"    MATRIX  {fortran_e(Q1[i][5], 10, 4)}  {fortran_e(Q1[i][6], 10, 4)}  {fortran_e(Q1[i][7], 10, 4)}{Q1[i][8]:8.3f}{Q1[i][9]:15.3f}")
            print(f"    LAMINA  {fortran_e(Q2[i][0], 10, 4)}  {fortran_e(Q2[i][1], 10, 4)}  {fortran_e(Q2[i][2], 10, 4)}{Q2[i][3]:8.3f}            -")

    # C     OUTPUT PLATE CONSTRUCTION
    if S['ISYM'] == 1: print(f"\n    SYMMETRIC LAMINATE{S['N1']:3d} LAYERS")
    if S['ISYM'] == 2: print(f"\n    ANTI-SYMMETRIC LAMINATE{S['N1']:3d} LAYERS")
    if S['ISYM'] == 3: print(f"\n    ASYMMETRIC LAMINATE{S['N1']:3d} LAYERS")
    
    print("\n    LAYER GEOMETRY\n")
    print("    LAYER        LAYER           LAYER                DISTANCE FROM")
    print("     No.         ANGLE         THICKNESS            REF. PLANE")
    for i in range(S['N1']):
        z_val = 0.0 if abs(Q[i][6]) < 1e-6 else Q[i][6]
        print(f"{i+1:6d}       {Q[i][0]:9.1f}         {fortran_e(Q[i][1], 10, 4)}          {fortran_e(z_val, 12, 4)}")
        
    print("\n    LAYER PROPERTIES\n")
    print("    LAYER      E(alpha)      E(beta)   G(alpha-beta)   Poisson's Ratio")
    print("     No                                                  (alpha-beta)")
    for i in range(S['N1']):
        print(f"{i+1:6d}     {fortran_e(Q[i][2], 12, 4)}   {fortran_e(Q[i][3], 11, 4)}  {fortran_e(Q[i][4], 11, 4)}    {Q[i][5]:9.3f}")

    # C     OUTPUT STIFFNESS SUB-MATRICES
    print("\n    IN-PLANE STIFFNESS SUB-MATRIX A\n")
    M8335(A)
    print("\n    COUPLED IN-PLANE AND FLEXURAL STIFFNESS SUB-MATRIX B\n")
    M8335(B)
    print("\n    FLEXURAL STIFFNESS SUB-MATRIX D\n")
    M8335(D)
    
    if S['X9'] > 0.0:
        print("\n    THE PLATE STIFFNESS MATRICES DO NOT SATISFY THE CONDITIONS\n    FOR SPECIAL ORTHOTROPY")
    else:
        print("\n    THE PLATE STIFFNESS MATRICES SATISFY THE CONDITIONS\n    FOR SPECIAL ORTHOTROPY")
        
    # C     OUTPUT APPARENT ELASTIC PROPERTIES
    print("\n    APPARENT ELASTIC PROPERTIES\n")
    print(f"    E(X)={fortran_e(S['E1'], 10, 4)}  E(Y)={fortran_e(S['E2'], 10, 4)}  G(XY)={fortran_e(S['E3'], 10, 4)}")
    print(f"    POISSON'S RATIO(XY)={S['E4']:6.4f}")
    print(f"    POISSON'S RATIO(YX)={S['E5']:6.4f}")
    print(f"    PLATE THICKNESS={fortran_e(S['H1'], 10, 4)}")


# C-----------------------------------------------------------------------
# C
#       SUBROUTINE M8335(AA)
# C
# C     PURPOSE
# C       TO OUTPUT A 3*3 MATRIX
# C
# C     LOCAL VARIABLES
# C       I,J - loop counters
def M8335(AA):
    for i in range(3):
        print("  " + "".join(f"  {fortran_e(AA[i][j], 11, 5)}" for j in range(3)))

if __name__ == '__main__':
    A8335(sys.argv[1] if len(sys.argv) > 1 else 'IN8335')