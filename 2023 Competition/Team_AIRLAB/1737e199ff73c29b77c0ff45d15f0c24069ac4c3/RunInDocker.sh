#!/usr/bin/env bash
echo "Only run this script in the root of your code base."
echo "Create Dockerfile using apt.txt"

# Specify which docker to use.
content="FROM ubuntu:jammy\nRUN apt-get update\n"

# Read packages to install.
pkgs='RUN ["apt-get", "install", "--yes", "--no-install-recommends"'
while read -r line;
do
   pkgs="${pkgs},\"$line\"" ;
done < apt.txt
pkgs="${pkgs}]\n"
content="${content}${pkgs}"

# Copy codes to target dir and set codes dir to be the working directory.
# Then run compile.sh to compile codes.
content="${content}COPY ./. /MAPF/codes/ \n"
content="${content}WORKDIR /MAPF/codes/ \n"
content="${content}RUN rm -rdf /MAPF/codes/build \n"
content="${content}RUN chmod u+x compile.sh \n"
content="${content}RUN ./compile.sh \n"
echo -e $content > Dockerfile

#!/usr/bin/env bash
echo "Only run this script in the root of your code base."
echo "Create Dockerfile using apt.txt"

# Specify which docker to use.
content="FROM ubuntu:jammy\nRUN apt-get update\n"

# Read packages to install.
pkgs='RUN ["apt-get", "install", "--yes", "--no-install-recommends"'
while read -r line;
do
   pkgs="${pkgs},\"$line\"" ;
done < apt.txt
pkgs="${pkgs}]\n"
content="${content}${pkgs}"

# Copy codes to target dir and set codes dir to be the working directory.
# Then run compile.sh to compile codes.
content="${content}COPY ./. /MAPF/codes/ \n"
content="${content}WORKDIR /MAPF/codes/ \n"
content="${content}RUN rm -rdf /MAPF/codes/build \n"
content="${content}RUN chmod u+x compile.sh \n"
content="${content}RUN ./compile.sh \n"
echo -e $content > Dockerfile

#echo "Remove container and images if exist... ..."
#out=$(docker container stop mapf_test 2>&1 ; docker container rm mapf_test 2>&1 ; docker rmi mapf_image 2>&1)

echo "Build image and run the container... ..."
# docker build --no-cache -t mapf_image ./
docker container run -it -v ./.:/MAPF/codes/ --name mapf_submit_test1 mapf_image
docker container run -it -v ./.:/MAPF/codes/ --name mapf_submit_test2 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test3 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test4 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test5 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test6 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test7 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test8 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test9 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test10 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test11 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test12 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test13 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test14 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test15 mapf_image
docker container run -d -it -v ./.:/MAPF/codes/ --name mapf_submit_test16 mapf_image