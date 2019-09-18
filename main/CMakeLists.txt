set(COMPONENT_SRCS "main.c" "normal_tasks.c" "data_sampling.c" "http_server.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "./components/modem/include")

set(COMPONENT_EMBED_FILES "favicon.ico" "ota_index.html" "index.html" "info.html")

register_component()

if(CONFIG_EXAMPLE_EMBEDDED_CERTS)
target_add_binary_data(${COMPONENT_TARGET} "certs/aws-root-ca.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/certificate.pem.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/private.pem.key" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/ca_cert.pem" TEXT)
endif()
