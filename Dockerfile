FROM eggeek01/gppc2021:gppc2021-base
ARG DEBIAN_FRONTEND=noninteractive

# Dependicies Installation
RUN apt-get update
RUN apt-get --yes --no-install-recommends install clang


# Copy codes to target dir and set codes dir to be the working directory.
# Then run compile.sh to compile codes.
COPY src /src
WORKDIR /src
RUN ls ./
RUN chmod u+x compile.sh
RUN ./compile.sh
RUN chmod u+x run
 
