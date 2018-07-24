all: gdpfs_tf_devel gdpfs_tf_gpu_devel gdpfs_hhcc_challenge

gdpfs_tf_devel:
	rm -f Dockerfile
	echo "FROM tensorflow/tensorflow:1.9.0-devel" > Dockerfile
	cat Dockerfile.template >> Dockerfile
	docker build -t $@ .
	rm -f Dockerfile

gdpfs_tf_gpu_devel:
	rm -f Dockerfile
	echo "FROM tensorflow/tensorflow:1.9.0-devel-gpu" > Dockerfile
	cat Dockerfile.template >> Dockerfile
	docker build -t $@ .
	rm -f Dockerfile

## this requires you run ./build-and-interact.sh and build the
## image 'docker-hhcc-challenge' that has ROS tools installed.
gdpfs_hhcc_challenge:
	rm -f Dockerfile
	echo "FROM docker-hhcc-challenge" > Dockerfile
	cat Dockerfile.template >> Dockerfile
	docker build -t $@ .
	rm -f Dockerfile
