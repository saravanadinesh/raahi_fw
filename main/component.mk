#
# Main Makefile. This is basically the same as a component makefile.
#

ifdef CONFIG_EXAMPLE_EMBEDDED_CERTS
# Certificate files. certificate.pem.crt & private.pem.key must be downloaded
# from AWS, see README for details.
COMPONENT_EMBED_TXTFILES := certs/aws-root-ca.pem certs/certificate.pem.crt certs/private.pem.key certs/ca_cert.pem
COMPONENT_EMBED_FILES := favicon.ico
COMPONENT_EMBED_FILES += ota_index.html
COMPONENT_EMBED_FILES += index.html
COMPONENT_EMBED_FILES += info.html

# Print an error if the certificate/key files are missing
$(COMPONENT_PATH)/certs/certificate.pem.crt $(COMPONENT_PATH)/certs/private.pem.key $(COMPONENT_PATH)/certs/aws-root-ca.pem $(COMPONENT_PATH)/certs/ca_cert.pem:
	@echo "Missing PEM file $@. This file identifies the ESP32 to AWS for the example, see README for details."
	exit 1
endif
