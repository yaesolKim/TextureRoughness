#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

void error_handling (char * message);

int main (int argc, char * argv[]) {
  int serv_sock, clnt_sock;

  struct sockaddr_in serv_adr;
  struct sockaddr_in clnt_adr;
  socklen_t clnt_adr_sz;

  int str_len;

  char message[] = "Hello, I'm Ubuntu, Server!";
  unsigned char message_rec[8];
  float pos_x, pos_y, pos_z;

  if (argc != 2) {
    printf("Usage : %s <port>\n", argv[0]);
    exit(1);
  }

  serv_sock = socket (PF_INET, SOCK_STREAM, 0); //create socket

  if (serv_sock == -1) {
    error_handling ("tcp socket creation error");
  }
  else {
    printf("success to creating tcp socket\n");
  }

  memset (&serv_adr, 0, sizeof(serv_adr));
  serv_adr.sin_family = AF_INET;
  serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_adr.sin_port = htons (atoi(argv[1]));

  //call 'bind' -> assign ip address and port number
  if (bind (serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
    error_handling ("bind() error");
  }
  else {
    printf("success to binding\n");
  }

  if(listen(serv_sock, 5) == -1) {
    error_handling("listen() error");
  }
  else {
    printf("success to listening\n");
  }

  clnt_adr_sz = sizeof(clnt_adr);
  clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_adr, &clnt_adr_sz); //call 'accept' -> accept connection

  if (clnt_sock == -1) {
    error_handling("accept() error");
  }

  printf("waiting for message from client\n");
  str_len = recv(clnt_sock, message_rec, 1023, 0);
  //str_len = read(clnt_sock, message_rec, 1023);
  message_rec[str_len] = 0;
  printf("message from client: %s\n", message_rec);

  printf("sending message to client\n");
  send(clnt_sock, message, sizeof(message), 0);


  while(1) {
    printf("waiting for message from client2\n");
    str_len = recv(clnt_sock, message_rec, 12, 0);
    memcpy(&pos_x, &message_rec, sizeof(pos_x));
    memcpy(&pos_y, &message_rec[4], sizeof(pos_y));
    memcpy(&pos_z, &message_rec[8], sizeof(pos_z));
    message_rec[str_len] = 0;
    printf("pos (x, y, z): (%f, %f, %f)\n", pos_x, pos_y, pos_z);
  }
  close(serv_sock);
  close(clnt_sock);
  return 0;
}

void error_handling (char * message) {
  fputs(message, stderr);
  fputc('\n', stderr);
  exit(1);
}
