FROM shugazine2017/server_base
RUN echo "ipv6" >> /etc/modules
RUN apk update && apk add nodejs

# Create app directory
RUN mkdir -p /usr/src/app
# Bundle app source
COPY . /usr/src/app

WORKDIR /usr/src/app

RUN echo "making MakeFile"
RUN cmake .
RUN echo "compiling"
RUN make

WORKDIR /usr/src/app/server
RUN mkdir uploaded
RUN chmod 777 resultImage
RUN npm install

CMD ["npm", "start"]