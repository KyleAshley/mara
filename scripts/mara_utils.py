

def inRange(num, lower, upper):
	return (num >= lower) and (num <= upper)

def isNearVal(num, val, tolerance):
	return (num >= val - tolerance) and (num <= val + tolerance)