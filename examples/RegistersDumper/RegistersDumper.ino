#include <MCP23017.h>
#include <MCP23S17.h>


MCP23017 mcp(&Wire);        // an MCP23017 connected to I2C
MCP23S17 mcp2(&SPI, SS);    // an MCP23S17 connected to SPI


/* Show all registers of a single instance. */
void
show_registers(IOExpander *dev, String name)
{
  uint8_t conf;

  Serial.print(name);
  Serial.println(F(" register dump:"));

  conf = dev->readRegister(MCP23017::IODIR_A);
  Serial.print("IODIR_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::IODIR_B);
  Serial.print("IODIR_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::IPOL_A);
  Serial.print("IPOL_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::IPOL_B);
  Serial.print("IPOL_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::GPINTEN_A);
  Serial.print("GPINTEN_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::GPINTEN_B);
  Serial.print("GPINTEN_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::DEFVAL_A);
  Serial.print("DEFVAL_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::DEFVAL_B);
  Serial.print("DEFVAL_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::INTCON_A);
  Serial.print("INTCON_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::INTCON_B);
  Serial.print("INTCON_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::IOCON);
  Serial.print("IOCON : ");
  Serial.print(conf, BIN);
  Serial.println();

//  conf = dev->readRegister(MCP23017::IOCONB);
//  Serial.print("IOCONB : ");
//  Serial.print(conf, BIN);
//  Serial.println();

  conf = dev->readRegister(MCP23017::GPPU_A);
  Serial.print("GPPU_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::GPPU_B);
  Serial.print("GPPU_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::INTF_A);
  Serial.print("INTF_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::INTF_B);
  Serial.print("INTF_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::INTCAP_A);
  Serial.print("INTCAP_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::INTCAP_B);
  Serial.print("INTCAP_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::GPIO_A);
  Serial.print("GPIO_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::GPIO_B);
  Serial.print("GPIO_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::OLAT_A);
  Serial.print("OLAT_A : ");
  Serial.print(conf, BIN);
  Serial.println();

  conf = dev->readRegister(MCP23017::OLAT_B);
  Serial.print("OLAT_B : ");
  Serial.print(conf, BIN);
  Serial.println();

  Serial.println();
}


void
setup()
{
  uint32_t tmr = millis();

  Serial.begin(115200);
  while (!Serial && ((millis() - tmr) < 5000))
        ;
  Serial.println();

  /* Initialize the I2C chip. */
  if (! mcp.begin(MCP23017_ADDRESS)) {
    Serial.println(F("MCP23017 not found !"));
  } else
    show_registers(&mcp, F("MCP23017"));

  /* Initialize the SPI chip. */
  if (! mcp2.begin(MCP23017_ADDRESS)) {
    Serial.println(F("MCP23S17 not found !"));
  } else
    show_registers(&mcp2, F("MCP23S17"));
}


void
loop()
{
  delay(1000);
}
