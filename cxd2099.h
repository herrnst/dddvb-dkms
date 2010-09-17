#ifndef _CXD2099_H_
#define _CXD2099_H_

#include <dvb_ca_en50221.h>

struct dvb_ca_en50221 *cxd2099_attach(u8 adr, void *priv, struct i2c_adapter *i2c);


#endif
