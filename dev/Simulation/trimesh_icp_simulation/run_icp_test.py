import os

input_dir="F:\\Data\\simulation\\data\\"
output_dir="F:\\Data\\simulation\\test_icp\\"
cabli_file="F:\\Data\\simulation\\data\\capture\\KK_left_pr"
exe="F:\\project\\bin\\x64\\Release\\trimesh_icp_simulation.exe"

def run_one_test(target, source, cabli, csv):
	print("run test :")
	os.system(exe + " testicp " + target + " " + source + " " + cabli + " " + csv)
	
for i in range(1, 20):
	target_index = i - 1
	source_index = i
	target_file=input_dir + str(target_index) + ".ply"
	source_file=input_dir + str(source_index) + ".ply"
	csv_file=output_dir + str(source_index) + "-" + str(target_index) + ".csv"
	run_one_test(target_file, source_file, cabli_file, csv_file)
	
	
