bash clean_examples.sh
clear

cd examples
for dir in *
do 
	cd ${dir} 
        make
	cd bdd
	make
        cd ..
	cd ..
done
