; SPDX-License-Identifier: GPL-3.0-only
; Own generic instruction expectations, independently assembled offline.
    org p:$1000
    movem b1,p:(r1)+
    movem b1,p:(r1)+
    move b1,x:(r1)+
    move b1,y:(r1)+
    rts
    move r1,a
