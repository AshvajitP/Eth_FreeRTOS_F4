#include <stdlib.h>
#include <string.h>

#include "Http_Open.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "transport_plaintext.h"

#include "core_http_config.h"
#include "core_http_client.h"
#include "transport_mbedtls.h"

#define HTTP_PORT 80
#define TRANSPORT_SEND_RECV_TIMEOUT_MS 3500
#define USER_BUFFER_LENGTH 1024
#define HTTP_NOT_SECURE 0
#define HOST_NAME "google.de"
#define HOST_URI "/get"
#define HTTPS_PORT 443
#define IOT_CORE_ALPN_PROTOCOL_NAME    "x-amzn-http-ca"

struct NetworkContext
{
#if HTTP_NOT_SECURE
    PlaintextTransportParams_t * pParams;
#else 
    TlsTransportParams_t * pParams;
#endif
};
uint8_t userBuffer[USER_BUFFER_LENGTH];
uint8_t response_buff[USER_BUFFER_LENGTH];
HTTPResponse_t my_response;
/* USER CODE BEGIN PV */
uint32_t FreeHeapSize_t = 0;
const char Root_CA_cert[] = SELF_SIGNED_ROOTCA;
const char Client_cert[] = CLIENT_CERT;
const char Client_key[] = CLIENT_KEY;


int32_t connectToServer(NetworkContext_t *pNetworkContext, const char *host, const unsigned int port,NetworkCredentials_t* xNetworkCredentials) {
BaseType_t xStatus = 0;
#if HTTP_NOT_SECURE
  PlaintextTransportStatus_t xNetworkStatus;
  BaseType_t xStatus = pdPASS;


    configASSERT( pNetworkContext != NULL );

    /* Establish a TCP connection with the HTTP server. This example connects to
     * the HTTP server as specified in democonfigSERVER_HOSTNAME and
     * democonfigHTTP_PORT in demo_config.h. */
    
    xNetworkStatus = Plaintext_FreeRTOS_Connect( pNetworkContext,
                                                 host,
                                                 port,
                                                 TRANSPORT_SEND_RECV_TIMEOUT_MS,
                                                 TRANSPORT_SEND_RECV_TIMEOUT_MS );

    if( xNetworkStatus != PLAINTEXT_TRANSPORT_SUCCESS )
    {
        xStatus = pdFAIL;
    }

    return xStatus;
    #else 
    FreeHeapSize_t = xPortGetFreeHeapSize();
    xStatus = TLS_FreeRTOS_Connect(pNetworkContext,host,port,
                                           xNetworkCredentials,
                                           TRANSPORT_SEND_RECV_TIMEOUT_MS,TRANSPORT_SEND_RECV_TIMEOUT_MS);
     if(xStatus != 0)
     {
       return pdFAIL;
     }
     else
     {
        return pdPASS;
     }
#endif
}
HTTPResponse_t request(const TransportInterface_t *pTransportInterface,
                                const char *pMethod,
                                size_t methodLen,
                                const char *pHost,
                                size_t hostLen,
                                const char *pPath,
                                size_t pathLen) {
  HTTPStatus_t httpStatus = HTTPSuccess;
  HTTPRequestInfo_t requestInfo = {0};
  HTTPResponse_t response = {0};
  HTTPRequestHeaders_t requestHeaders = {0};

  requestInfo.pMethod = pMethod;
  requestInfo.methodLen = methodLen;
  requestInfo.pHost = pHost;
  requestInfo.hostLen = hostLen;
  requestInfo.pPath = pPath;
  requestInfo.pathLen = pathLen;
  requestInfo.reqFlags = HTTP_REQUEST_KEEP_ALIVE_FLAG;

  requestHeaders.pBuffer = userBuffer;
  requestHeaders.bufferLen = USER_BUFFER_LENGTH;

  httpStatus = HTTPClient_InitializeRequestHeaders(&requestHeaders, &requestInfo);

  if (httpStatus == HTTPSuccess) {
    my_response.pBuffer = response_buff;
    my_response.bufferLen = USER_BUFFER_LENGTH;

    httpStatus = HTTPClient_Send( pTransportInterface,
                                  &requestHeaders,
                                  0, // ( uint8_t * ) REQUEST_BODY,
                                  0, // REQUEST_BODY_LENGTH,
                                  &my_response,
                                  0 );
  } else {
   /* LogError(("Failed to initialize HTTP request headers: Error=%s.", HTTPClient_strerror(httpStatus)));*/
  }

  return response;
}

void http_get() {
  int32_t returnStatus = pdTRUE;
  BaseType_t returnstat;
 // typedef struct NetworkContext NetworkContext_t ;
  NetworkContext_t networkContext = {0};
    TransportInterface_t transportInterface = {0};
#if HTTP_NOT_SECURE 
  PlaintextTransportParams_t plaintextParams;
    networkContext.pParams = &plaintextParams;
 while( pdPASS != connectToServer(&networkContext, HOST_NAME, HTTP_PORT) )
            {
                vTaskDelay( pdMS_TO_TICKS( 5000U ) );
            }
  transportInterface.pNetworkContext = &networkContext;
  transportInterface.recv = Plaintext_FreeRTOS_recv;
  transportInterface.send = Plaintext_FreeRTOS_send;
#else 
  TlsTransportParams_t xTlsTransportParams = { 0 };
  networkContext.pParams = &xTlsTransportParams;
  
  NetworkCredentials_t xNetworkCredentials = { 0 };
    static const char * pcAlpnProtocols[] = { IOT_CORE_ALPN_PROTOCOL_NAME, NULL };
    xNetworkCredentials.pAlpnProtos = pcAlpnProtocols;

xNetworkCredentials.disableSni = pdTRUE;
/* Set the credentials for establishing a TLS connection. */
xNetworkCredentials.pRootCa = ( const unsigned char * ) rootCA_Certificate;
xNetworkCredentials.rootCaSize = strlen( rootCA_Certificate );
xNetworkCredentials.pClientCert = ( const unsigned char * ) Client_cert;
xNetworkCredentials.clientCertSize = strlen( Client_cert );
xNetworkCredentials.pPrivateKey = ( const unsigned char * ) Client_key;
xNetworkCredentials.privateKeySize = strlen( Client_key );
   while( pdPASS != connectToServer(&networkContext,HOST_NAME, HTTPS_PORT, &xNetworkCredentials) )
            {
                vTaskDelay( pdMS_TO_TICKS( 5000U ) );
            }
  transportInterface.pNetworkContext = &networkContext;
 transportInterface.recv = TLS_FreeRTOS_recv;
  transportInterface.send = TLS_FreeRTOS_send;
#endif
  
  uint16_t host_name_len = strlen(HOST_NAME);
  uint8_t path_len = strlen(HOST_URI);
   
  HTTPResponse_t response = request(&transportInterface, "GET", 3, HOST_NAME, host_name_len, HOST_URI,path_len);

  TLS_FreeRTOS_Disconnect(&networkContext);
}
