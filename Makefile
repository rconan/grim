crate:
	cd .. ;  git archive --format=tar.gz HEAD > docker/crate.tar.gz
build: crate.tar.gz
	docker build -t gmto.im/wind2optics .
run:
	docker run --rm --env-file config.txt gmto.im/wind2optics
push:
	aws ecr get-login-password --region us-west-1 | docker login --username AWS --password-stdin 378722409401.dkr.ecr.us-west-1.amazonaws.com
	docker tag gmto.im/wind2optics:latest 378722409401.dkr.ecr.us-west-1.amazonaws.com/gmto.im/wind2optics:latest
	docker push 378722409401.dkr.ecr.us-west-1.amazonaws.com/gmto.im/wind2optics:latest
stack:
	aws s3 cp ../wind2optics.yaml s3://gmto.modeling/stacks/
	aws cloudformation create-stack --stack-name wind2optics --template-url https://s3-us-west-2.amazonaws.com/gmto.modeling/stacks/wind2optics.yaml --region us-west-1

