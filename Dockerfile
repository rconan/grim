FROM nvidia/cuda:10.1-devel as build

# Update default packages
RUN apt-get update  

# Get Ubuntu packages
RUN apt-get install -y \
    build-essential \
    curl cmake libfreetype6 libfreetype6-dev \
    libclang-dev libcurl4-openssl-dev

# Update new packages
RUN apt-get update

# Get Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

ENV PATH="/root/.cargo/bin:${PATH}"

ADD crseo.tgz .
ADD dos-actors.tgz dos-actors

RUN USER=root cargo new --bin grim
WORKDIR /grim

RUN ls ../crseo/CEO/lib/

COPY ./Cargo.toml ./Cargo.toml
COPY modal_state_space_model_2ndOrder.zip /grim/
ENV FEM_REPO=/grim
COPY ./src ./src
RUN touch /crseo/build.rs
RUN cargo build --release --verbose

RUN ls target/release/
