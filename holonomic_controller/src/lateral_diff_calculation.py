
from math import *

## due to surface the lateral travelled distance will not equal to reality
## but the linear one will be equal reality
## to solve this lateral mismatch to reality we need to multiply vy with multiplication facotr

## check this link for reference
# https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiBp6D2p5b5AhX8SWwGHdJjDMoQFnoECAYQAQ&url=https%3A%2F%2Fcore.ac.uk%2Fdownload%2Fpdf%2F12525654.pdf&usg=AOvVaw1WNMEsLV4UwsBoDeiWFx_E


## update here your reference linar and lateral value 
one_meter_linear_distance = 1.2
one_meter_lateral_distance = 1.27

alpha = degrees(atan(one_meter_linear_distance/one_meter_lateral_distance))

vy_multiplication_factor = tan(radians(alpha))

## here you will get an output (the multiplication factor)
print(vy_multiplication_factor)
