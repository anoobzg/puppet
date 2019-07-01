import sys,os,shutil
import random

def mapping(lists):
	l = len(lists)
	b = random.randint(0, l - 1)
	return lists[b]
	
offset = 300
lists = []
for i in range(300, 400):
	lists.append(i)
	
for i in range(0, 100):
	source_file = str(i)+".ply"
	dest_index = mapping(lists)
	dest_file = str(dest_index) + ".ply"
	lists.remove(dest_index)
	if os.path.isfile(source_file):
		shutil.copyfile(source_file, dest_file)