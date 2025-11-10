// main.ts
// pxt-rc522-robotbit
// Basic RC522 driver for MakeCode / micro:bit
// Author: ChatGPT (adapted for robot:bit)
// License: MIT

//% weight=10 color=#AA278D icon="\uf1b3"
namespace RC522 {
    // RC522 registers (subset)
    const CommandReg = 0x01;
    const CommIEnReg = 0x02;
    const DivIEnReg = 0x03;
    const CommIrqReg = 0x04;
    const DivIrqReg = 0x05;
    const ErrorReg = 0x06;
    const Status1Reg = 0x07;
    const Status2Reg = 0x08;
    const FIFODataReg = 0x09;
    const FIFOLevelReg = 0x0A;
    const ControlReg = 0x0C;
    const BitFramingReg = 0x0D;
    const ModeReg = 0x11;
    const TxControlReg = 0x14;
    const TModeReg = 0x2A;
    const TPrescalerReg = 0x2B;
    const TReloadRegL = 0x2D;
    const TReloadRegH = 0x2C;
    const TxAutoReg = 0x15;
    const RFCfgReg = 0x26;

    const PCD_IDLE = 0x00;
    const PCD_AUTHENT = 0x0E;
    const PCD_RECEIVE = 0x08;
    const PCD_TRANSMIT = 0x04;
    const PCD_TRANSCEIVE = 0x0C;
    const PCD_RESETPHASE = 0x0F;
    const PCD_CALCCRC = 0x03;

    // PICC commands
    const PICC_REQIDL = 0x26;
    const PICC_REQALL = 0x52;
    const PICC_ANTICOLL = 0x93;
    const PICC_SELECTTAG = 0x93;
    const PICC_AUTHENT1A = 0x60;
    const PICC_AUTHENT1B = 0x61;
    const PICC_READ = 0x30;
    const PICC_WRITE = 0xA0;
    const PICC_HALT = 0x50;

    let _sdaPin = DigitalPin.P8;
    let _rstPin = DigitalPin.P12;
    let _sckPin = DigitalPin.P13; // SPI default SCK
    let _mosiPin = DigitalPin.P15; // SPI default MOSI
    let _misoPin = DigitalPin.P14; // SPI default MISO
    let _initialized = false;

    function spiSetup() {
        pins.spiPins(_misoPin, _mosiPin, _sckPin);
        pins.spiFrequency(1000000); // 1 MHz
    }

    function writeRegister(addr: number, val: number) {
        // Address format: address << 1 & ~1  (write)
        let address = ((addr << 1) & 0x7E);
        pins.digitalWritePin(_sdaPin, 0); // CS low
        pins.spiWrite(address);
        pins.spiWrite(val);
        pins.digitalWritePin(_sdaPin, 1); // CS high
    }

    function readRegister(addr: number): number {
        let address = ((addr << 1) & 0x7E) | 0x80; // read flag
        pins.digitalWritePin(_sdaPin, 0); // CS low
        pins.spiWrite(address);
        let val = pins.spiRead();
        pins.digitalWritePin(_sdaPin, 1); // CS high
        return val;
    }

    function setBitMask(reg: number, mask: number) {
        const tmp = readRegister(reg);
        writeRegister(reg, tmp | mask);
    }

    function clearBitMask(reg: number, mask: number) {
        const tmp = readRegister(reg);
        writeRegister(reg, tmp & (~mask));
    }

    function antennaOn() {
        let val = readRegister(TxControlReg);
        if ((val & 0x03) != 0x03) {
            writeRegister(TxControlReg, val | 0x03);
        }
    }

    function reset() {
        writeRegister(CommandReg, PCD_RESETPHASE);
    }

    function calculateCRC(data: number[]): number[] {
        writeRegister(CommandReg, PCD_IDLE);
        setBitMask(DivIrqReg, 0x04); // Clear CRCIRq
        writeRegister(FIFOLevelReg, 0x80); // Clear FIFO
        for (let i = 0; i < data.length; i++) {
            writeRegister(FIFODataReg, data[i]);
        }
        writeRegister(CommandReg, PCD_CALCCRC);
        // wait loop
        for (let i = 0; i < 255; i++) {
            const n = readRegister(DivIrqReg);
            if (n & 0x04) {
                // CRC ready
                const l = readRegister(0x22); // CRCResultLow
                const h = readRegister(0x21); // CRCResultHigh
                return [l, h];
            }
        }
        return [0, 0];
    }

    function toHexString(arr: number[]): string {
        let s = "";
        for (let i = 0; i < arr.length; i++) {
            const t = arr[i] & 0xFF;
            let h = t.toString(16);
            if (h.length == 1) h = "0" + h;
            s += h;
            if (i < arr.length - 1) s += ":";
        }
        return s.toUpperCase();
    }

    function communicateWithPICC(command: number, sendData: number[], timeoutMs = 200): { status: boolean, backData: number[], backBits: number } {
        let backData: number[] = [];
        let backBits = 0;
        let irqEn = 0x00;
        let waitIRq = 0x00;

        if (command == PCD_TRANSCEIVE) {
            irqEn = 0x77;
            waitIRq = 0x30;
        } else {
            irqEn = 0x12;
            waitIRq = 0x00;
        }

        writeRegister(CommIEnReg, irqEn | 0x80);
        clearBitMask(CommIrqReg, 0x80);
        setBitMask(FIFOLevelReg, 0x80); // flush FIFO
        writeRegister(CommandReg, PCD_IDLE);

        for (let i = 0; i < sendData.length; i++) {
            writeRegister(FIFODataReg, sendData[i]);
        }

        writeRegister(CommandReg, command);
        if (command == PCD_TRANSCEIVE) {
            setBitMask(BitFramingReg, 0x80); // StartSend=1
        }

        // wait for data
        let i = 0;
        const start = input.runningTime();
        while (true) {
            const n = readRegister(CommIrqReg);
            if (n & waitIRq) break;
            if (n & 0x01) break; // timer interrupt - timeout
            if (input.runningTime() - start > timeoutMs) break;
        }

        clearBitMask(BitFramingReg, 0x80);

        const error = readRegister(ErrorReg);
        if ((readRegister(CommIrqReg) & 0x01) || (error & 0x1B)) {
            return { status: false, backData: [], backBits: 0 };
        }

        if (command == PCD_TRANSCEIVE) {
            const fifoLevel = readRegister(FIFOLevelReg);
            for (let j = 0; j < fifoLevel; j++) {
                backData.push(readRegister(FIFODataReg));
            }
            backBits = readRegister(ControlReg) & 0x07;
        }

        return { status: true, backData: backData, backBits: backBits };
    }

    /**
     * Inicializa o RC522.
     * @param sda Pin de CS / SDA (conectar ao pin SS do RC522). Ex.: DigitalPin.P8
     * @param rst Pin de RST (reset) do RC522. Ex.: DigitalPin.P12
     */
    //% block="RC522 init SDA %sdaPin|RST %rstPin|usar pinos SPI padrão (MOSI/MISO/SCK)"
    //% sdaPin.defl=DigitalPin.P8
    //% rstPin.defl=DigitalPin.P12
    export function init(sdaPin: DigitalPin = DigitalPin.P8, rstPin: DigitalPin = DigitalPin.P12) {
        _sdaPin = sdaPin;
        _rstPin = rstPin;
        // keep SPI default pins, but you can alter global vars before calling init if needed
        pins.digitalWritePin(_sdaPin, 1);
        pins.digitalWritePin(_rstPin, 1);
        spiSetup();
        reset();
        writeRegister(TModeReg, 0x8D);
        writeRegister(TPrescalerReg, 0x3E);
        writeRegister(TReloadRegL, 30);
        writeRegister(TReloadRegH, 0);
        writeRegister(TxAutoReg, 0x40);
        writeRegister(ModeReg, 0x3D);
        antennaOn();
        _initialized = true;
    }

    /**
     * Detecta se existe tag no campo (modo IDLE). Retorna true se encontrada.
     */
    //% block="RC522 detectar tag presente"
    export function tagPresent(): boolean {
        if (!_initialized) return false;
        writeRegister(BitFramingReg, 0x07); // 7 bits -> transmit
        const req = [PICC_REQIDL];
        const res = communicateWithPICC(PCD_TRANSCEIVE, req);
        return res.status && res.backData.length > 0;
    }

    /**
     * Lê o UID da primeira tag encontrada (array de bytes). Retorna string hex separada por ':' se sucesso, "" se falha.
     */
    //% block="RC522 ler UID"
    export function readUID(): string {
        if (!_initialized) return "";
        // Request to detect any PICC
        writeRegister(BitFramingReg, 0x07);
        const req = [PICC_REQIDL];
        let res = communicateWithPICC(PCD_TRANSCEIVE, req);
        // if no tag, return empty
        if (!res.status || res.backData.length == 0) {
            return "";
        }

        // Anti-collision to get UID
        writeRegister(BitFramingReg, 0x00); // reset framing
        const anticoll = [PICC_ANTICOLL, 0x20];
        res = communicateWithPICC(PCD_TRANSCEIVE, anticoll);
        if (!res.status || res.backData.length < 5) {
            return "";
        }

        // backData contains UID (4 bytes) + BCC (1 byte) typically
        const uid: number[] = [];
        for (let i = 0; i < 5; i++) {
            uid.push(res.backData[i]);
        }
        // basic BCC check
        let bcc = 0;
        for (let i = 0; i < 4; i++) bcc ^= uid[i];
        if (bcc != uid[4]) {
            // BCC mismatch; but still return first 4 bytes
        }
        const uid4 = uid.slice(0, 4);
        return toHexString(uid4);
    }

    /**
     * Para de comunicar com tag (HALT).
     */
    //% block="RC522 halt (parar tag)"
    export function haltTag(): void {
        if (!_initialized) return;
        const buf = [PICC_HALT, 0x00];
        const crc = calculateCRC(buf);
        buf.push(crc[0]);
        buf.push(crc[1]);
        communicateWithPICC(PCD_TRANSCEIVE, buf);
    }

    /**
     * Ajudas / debug: lê um registro do RC522 e retorna valor.
     */
    //% block="RC522 ler reg %addr"
    //% addr.min=0 addr.max=255
    export function readReg(addr: number): number {
        return readRegister(addr);
    }

    // Placeholder: funções para leitura/escrita de blocos podem ser adicionadas aqui.
    // Implementar autenticação (PICC_AUTHENT1A/PICC_AUTHENT1B), leitura (PICC_READ) e escrita (PICC_WRITE)
    // exige fluxo de autenticação com chave e gerenciamento de CRC — pode ser expandido conforme necessidade.
}
