FROM nvidia/cuda:10.1-devel-ubuntu18.04 as build

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
RUN apt-get update && apt-get install -y git curl libcurl4-openssl-dev\
    build-essential llvm-dev libclang-dev clang noweb

RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

ADD Cargo.toml /test/
ADD src/main.rs /test/src/
WORKDIR /test 

RUN git clone -b rust https://github.com/rconan/ceo.git
RUN cd ceo && make all install
RUN cargo build --release --bin main

FROM nvidia/cuda:10.1-runtime-ubuntu18.04

ADD ceo-modes.tar .
ENV GMT_MODES_PATH="/"

COPY --from=build /test/target/release/main grim

CMD ["./grim"]