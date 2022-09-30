build:
	docker build -t gmto.im/grim .
run:
	docker run --gpus all --rm gmto.im/grim
push:
	aws ecr get-login-password --region us-west-2 | docker login --username AWS --password-stdin 378722409401.dkr.ecr.us-west-2.amazonaws.com
	docker tag gmto.im/grim:latest 378722409401.dkr.ecr.us-west-2.amazonaws.com/gmto.im/grim:latest
	docker push 378722409401.dkr.ecr.us-west-2.amazonaws.com/gmto.im/grim:latest
stack:
	aws s3 cp grim.yaml s3://gmto.modeling/stacks/
	aws cloudformation create-stack --stack-name grim --template-url https://s3-us-west-2.amazonaws.com/gmto.modeling/stacks/grim.yaml --region us-west-2
