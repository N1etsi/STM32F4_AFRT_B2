# PID Tunning (ref Joop Brokking)

1. Yaw tunning
    Set P to 3 
    Set I to 0.02
    No D gain
    //provisional values

2. Roll and Pitch
    a) Start with D gain:
        D = 5
        Increase 1 by 1 until it becomes restless
        Find last value it doesn't oscilate
        Reduce that value by 25%
    
    b) Now the P gain:
        P = 0.2
        Increase by 0.2 until it starts to overcompensate in low flying
        Reduce that value by 50%

    c) Now the I gain:
        I = 0.01
        Increase by 0.01 until it starts to slowly oscilate
        Reduce by 50%

    e)Back to the P gain:
        Increase by 0.1 or 0.05until fast oscilation
        Reduce value by 90%/85%


