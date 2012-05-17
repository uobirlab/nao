# A message to set Nao's LED groups. Chest and eyes can be set with RGB values. Head and ears are blue, so the blue value is interpreted as an intensity.

uint8 led   # which group to set (head, both ears, both eyes, chest, both feet) - more fine-grained control is possible if necessary

uint8 red   # red value (0 - 255)
uint8 green   # green value (0 - 255)
uint8 blue   # blue value (0 - 255)

duration duration # how long to take to change to the new value

# the only valid values of led are:

uint8 ledBothEars     = 0
uint8 ledBothEyes     = 1
uint8 ledHead         = 2
uint8 ledChest        = 3
uint8 ledBothFeet     = 4
uint8 ledAll          = 5