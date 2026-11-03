/* SPDX-License-Identifier: MIT */
#ifndef RECONIC_NETDEV_H
#define RECONIC_NETDEV_H

struct edev_dev;
struct pci_dev;
struct reconic_netdev;

int reconic_netdev_register(struct edev_dev *edev, struct pci_dev *pdev,
                            struct reconic_netdev **out);
void reconic_netdev_unregister(struct reconic_netdev *rdev);

#endif
