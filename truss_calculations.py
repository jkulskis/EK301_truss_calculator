from mat4py import loadmat
import numpy as np
import datetime

filename = "example_input.mat"

def buckle_force(length):
	"""Calculates the buckle force for a straw depending on the length
	
	Arguments:
		length {double} -- length of the straw
	"""
	return 1327.9/length**2  

def print_array(arr):
	"""Prints a 2D array out cleanly
	"""
	for row in arr:
		for val in row:
			print('' + str(round(val, 2)), end='\t')
		print('')
	print('')

if __name__ == '__main__':
	member_lengths = []
	try:
		in_vals = loadmat(filename)
		C = np.array(in_vals['C'])
		Sx = np.array(in_vals['Sx'])
		Sy = np.array(in_vals['Sy'])
		X = np.array(in_vals['X'])
		Y = np.array(in_vals['Y'])
		L = np.array(in_vals['L'])
		print(f"Succesfully loaded from the input file \"{filename}\"\n")
	except:
		print(
			f"Failed to load from the input file \"{filename}\" so using test files instead\n")
		C = np.array([[1, 1, 0, 0, 0, 0, 0], [1, 0, 1, 0, 1, 1, 0], [
			0, 1, 1, 1, 0, 0, 0], [0, 0, 0, 1, 1, 0, 1], [0, 0, 0, 0, 0, 1, 1]])
		Sx = np.array([[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
		Sy = np.array([[0, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 1]])
		X = np.array([1, 5, 4, 1, 2])
		# don't make X/Y all zeroes to avoid dividing by zero
		Y = np.array([8, 2, 11, 3, 10])
		L = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [1], [0]])

	########### Check some values real quick ############
	num_members = -1
	for row in C:
		if num_members == -1:
			num_members = len(row)
		assert(len(row) == num_members)  # All rows should be the same length
	for row in Sx:
		assert(len(row) == 3)  # Should be length of 3 for each row
	for row in Sy:
		assert(len(row) == 3)  # Should be length of 3 for each row
	for row in L:
		assert(len(row) == 1)  # Should be length of 1 for each row
	num_joints = len(C)
	assert(len(X) == len(Y) == num_joints)  # Should be same number of x and y coords as joints
	#####################################################

	load = sum(val[0] for val in L) # load is equal to the sum of all of the L forces

	# cost is defined as cost = $10J + $1L, where J is the number of joints and L is the total
	# of all the straw lengths summed together in cm
	cost = 10*num_joints + sum([val[0] for val in L])

	A = []  # start off with a python list before converting to a numpy array
	# empty 2D list to store x forces
	X_forces = [[0]*num_members for ii in range(num_joints)]
	# empty 2D list to store y forces
	Y_forces = [[0]*num_members for ii in range(num_joints)]

	con_joints = [-1, -1]

	# go through each member...length of C[0] should be the same for any index of C (checked above with asserts)
	for ii in range(num_members):
		for jj in range(num_joints):  # go through each joint to find the 2 connected ones
			if (C[jj][ii]):  # if the joint is connected to this member
				if con_joints[0] == -1:
					con_joints[0] = jj
				else:
					con_joints[1] = jj
					break
		dist = ((X[con_joints[0]]-X[con_joints[1]])**2+(Y[con_joints[0]]-Y[con_joints[1]])**2)**(0.5)
		member_lengths.append(dist) # add the distance to the member lengths to calculate the failing member later
		X_forces[con_joints[0]][ii] = (X[con_joints[1]]-X[con_joints[0]])/dist 
		X_forces[con_joints[1]][ii] = (X[con_joints[0]]-X[con_joints[1]])/dist
		Y_forces[con_joints[0]][ii] = (Y[con_joints[1]]-Y[con_joints[0]])/dist 
		Y_forces[con_joints[1]][ii] = (Y[con_joints[0]]-Y[con_joints[1]])/dist  
		con_joints = [-1, -1]

	for ii in range(num_joints):
		A.append(X_forces[ii] + list(Sx[ii]))
	for ii in range(num_joints):
		A.append(Y_forces[ii] + list(Sy[ii]))
	A = np.array(A) # By this point A should be filled correctly

	T = np.matmul(np.linalg.inv(A), L)

	fail_ratio = -1
	for ii in range(num_members):
		ratio = abs(T[ii][0])/buckle_force(member_lengths[ii])
		if ratio > fail_ratio:
			fail_ratio = ratio
			fail_index = ii

	########### DEBUG PRINTS ###########
	# print_array(X_forces) 
	# print_array(Y_forces)
	# print_array(A)
	# print_array(T)
	####################################

	############## OUTPUT OF RESULTS ###############

	print(f"EK301, Section A3, Group GoonSquad: Garrett G, John M, Mark T {datetime.datetime.now().date()}")

	print(f"\nLoad: {load} N")

	print("\nMember Forces in Newtons:")
	for ii in range(num_members):
		print(f"m{ii+1}: {abs(round(T[ii][0], 3))} {'(C)' if T[ii] < 0 else '(T)'}")

	print("\nReaction Forces in Newtons:")
	print(f"Sx1: {round(T[num_members][0], 2)}")
	print(f"Sy1: {round(T[num_members+1][0], 2)}")
	print(f"Sy2: {round(T[num_members+2][0], 2)}")

	print(f"\nCost of Truss: ${cost}")
	print(f"Theoretical max load/cost ratio in N/$: {round(load/cost, 4)}")
	
	print(f"\nFailing member: m{fail_index}")
	print(f"Maximum Theoretical Load: {round(load/fail_ratio, 3)} N")
