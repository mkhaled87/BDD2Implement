cd examples
for dir in *
do 
	cd ${dir} 
        make clean
	cd bdd
		make clean
        cd ..
	cd ..
done
