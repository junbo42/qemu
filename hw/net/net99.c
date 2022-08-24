/*
 * QEMU educational network device.
 *
 * Copyright (c) 2022 Junbo Jiang
 *
 * This work is licensed under the terms of the GNU GPL, version 2.
 *
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/qdev-properties.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/pci/pci.h"
#include "net/net.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qapi/visitor.h"

#define TYPE_PCI_NET99_DEVICE "net99"

typedef struct Net99State Net99State;

DECLARE_INSTANCE_CHECKER(Net99State, NET99, TYPE_PCI_NET99_DEVICE)

struct Net99State {
    PCIDevice pdev;
    NICConf conf;
    qemu_irq irq;
    NICState *nic;

    uint32_t rx_addr;
    uint32_t tx_addr;
    uint8_t rx_idx;
    uint8_t rx_cur;
    uint8_t tx_idx;
    uint8_t tx_cur;
    int intr_status;

    MemoryRegion io;
    MemoryRegion mmio;
};

//static bool net99_can_receive(NetClientState *nc){
//    Net99State *s = qemu_get_nic_opaque(nc);
//    bool ret = true;
//    if(s->intr_status)
//        ret = false;
//
//    return ret;
//}

static ssize_t net99_receive(NetClientState *nc, const uint8_t *buf, size_t size){
    Net99State *s = qemu_get_nic_opaque(nc);

    if(s->rx_addr){
        uint64_t ps;
        int offset;

        s->rx_idx++;

        offset = s->rx_idx * (1514 + 8);

        pci_dma_read(PCI_DEVICE(s), s->rx_addr + offset, &ps, 8);
        if(!ps){
            pci_dma_write(PCI_DEVICE(s), s->rx_addr + offset, &size, 8);
            pci_dma_write(PCI_DEVICE(s), s->rx_addr + offset + 8, buf, size);
        }else{
            s->rx_idx--;
        }

        //qemu_set_irq(s->irq, 1);
        pci_set_irq(PCI_DEVICE(s), 1);

        s->intr_status = 1;
    }

    return size;
}

static void net99_transmit(Net99State *s){
    uint8_t buf[1514];
    uint8_t idx;
    uint64_t size;

    idx = s->tx_idx;

    pci_dma_read(PCI_DEVICE(s), s->tx_addr + (1514 + 8) * idx, &size, 8);

    if(size <= 1514){
        pci_dma_read(PCI_DEVICE(s), s->tx_addr + (1514 + 8) * idx + 8, buf, size);

        qemu_send_packet(qemu_get_queue(s->nic), buf, size);
    }
}

enum net99_io_addr {
    INTR_STATUS = 0,
    RX_ADDR     = 4,
    RX_IDX      = 8,
    RX_CUR      = 12,
    RX_RST      = 16,
    TX_ADDR     = 20,
    TX_IDX      = 24,
    TX_CUR      = 28,
    TX_RST      = 32,
};

static uint64_t net99_io_read(void *opaque, hwaddr addr, unsigned size){
    Net99State *s = opaque;
    uint32_t ret = -1;

    switch(addr){
    case INTR_STATUS:
        ret = s->intr_status;
        //qemu_set_irq(s->irq, 0);
        pci_set_irq(PCI_DEVICE(s), 0);
        s->intr_status = 0;
        break;
    case RX_IDX:
        ret = s->rx_idx;
        break;
    case RX_CUR:
        ret = s->rx_cur;
        break;
    case TX_IDX:
        ret = s->tx_idx;
        break;
    case TX_CUR:
        ret = s->tx_cur;
        break;
    default:
    }

    return ret;
}

static void net99_io_write(void *opaque, hwaddr addr, uint64_t data, unsigned size){ 
    Net99State *s = opaque;

    char p[32] = {0};

    switch(addr){
    case RX_ADDR:
        s->rx_addr = data;
        pci_dma_read(PCI_DEVICE(s), s->rx_addr, p, 5);
        break;
    case RX_CUR:
        s->rx_cur = data;
        break;
    case RX_RST:
        s->rx_idx = -1;
        s->intr_status = 0;
        break;
    case TX_ADDR:
        s->tx_addr = data;
        break;
    case TX_IDX:
        s->tx_idx = data;
        net99_transmit(s);
        break;
    default:
    }
}

static const MemoryRegionOps net99_io_ops = {
    .read = net99_io_read,
    .write = net99_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static NetClientInfo net_net99_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    //.can_receive = net99_can_receive,
    .receive = net99_receive,
};

static void pci_net99_realize(PCIDevice *pdev, Error **errp)
{
    Net99State *s = NET99(pdev);
    uint8_t *pci_conf;

    pci_conf = pdev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 1;

    memory_region_init_io(&s->io, OBJECT(pdev), &net99_io_ops, s, "net99", 0x80);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io);

    //qemu_macaddr_default_if_unset(&s->conf.macaddr);

    s->irq = pci_allocate_irq(pdev);

    s->nic = qemu_new_nic(&net_net99_info, &s->conf,
                          object_get_typename(OBJECT(pdev)),
                          pdev->qdev.id, s);
}

//static void net99_instance_init(Object *obj)
//{
//    //Net99State *ndev = NET99(obj);
//}
//
//static void pci_net99_uninit(PCIDevice *pdev)
//{
//    //Net99State *ndev = NET99(pdev);
//}

static Property net99_properties[] = {                                                                                                                                   
    DEFINE_NIC_PROPERTIES(Net99State, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void net99_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_net99_realize;
    //k->exit = pci_net99_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = 0x0099;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;

    device_class_set_props(dc, net99_properties);

    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo net99_dev_info = {
    .name          = TYPE_PCI_NET99_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Net99State),
    //.instance_init = net99_instance_init,
    .class_init    = net99_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE},
        { }}
};

static void pci_net99_register_types(void)
{
    type_register_static(&net99_dev_info);
}

type_init(pci_net99_register_types)
