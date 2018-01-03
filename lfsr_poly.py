polynomial = [16,15,13,4]
value = 0
for coef in polynomial:
	value += 1<<(coef-1)
print (bin(value))
print (hex(value))