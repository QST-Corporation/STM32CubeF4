# -*- coding: utf-8 -*-
  
# File Name : FLASH_QST_hexf_to_byte_array.py
# Version : V0.01
# Date : 2021/01/26
# Author : QST AE team

import os
import time
from functools import reduce

# get '.hexf' file name from current path
def getFileNamebyEX(path):
    f_list = os.listdir(path)
    for i in f_list:
        filename = os.path.splitext(i)[0]
        extname = os.path.splitext(i)[1]
        if extname == '.hexf':
            print(" *Hexf file : " + filename + extname+'*')
            return i

bin_buf1 = []# raw data of binary is to be stored here
bin_buf2 = []
bin_buf3 = []
str_buf1 = []
str_buf2 = []
str_buf3 = []

def hex2str(hex_file_name,h_file_name):
    with open(hex_file_name,'r') as frd:
        print(' *Hexf file : \''+hex_file_name+'\''+' is opened*')
        byte_num = 0
        bin_num = 0
        for line in frd.readlines():
            if(line[0] == ':'):
                if(line[7:9] == '04'):               #Extended Linear Address Record
                   #print('Extended Linear Address Record');
                   line = char2hex(line)
                   if checksum(line) == 0:         #checksum passed
                       addr_h = (line[4]<<24) +(line[5]<<16)
                       byte_num = 0
                       bin_num += 1
                       print(" ------------------------------")
                       print(" *File bin%d:" %bin_num)
                       if bin_num == 2:
                            write_str(str_buf1, 1, addr_start)
                            bin1_chksum = reduce(lambda x,y:x+y,bin_buf1[:])
                            bin_info = "const char bin1_chksum[] = \"%x"%bin1_chksum + "\";\n"
                       elif bin_num == 3:
                            write_str(str_buf2, 2, addr_start)
                            bin2_chksum = reduce(lambda x,y:x+y,bin_buf2[:])
                            bin_info += "const char bin2_chksum[] = \"%x"%bin2_chksum + "\";\n"
                   else:
                       print('checksum failed!'+str(list(map(hex,line))))
                elif (line[7:9] == '00'):          #Data Record
                    strline = line[9:-3]
                    line = char2hex(line)
                    if checksum(line) == 0:
                        addr_l = (line[1]<<8) + line[2]
                        LL = line[0]
                        byte_num = byte_num + LL
                        #print('byte_num %d' %byte_num)
                        for data in strline[:]: #for data in line[4:-1]:
                            if bin_num == 1:
                                str_buf1.append(data)
                            elif bin_num == 2:
                                str_buf2.append(data)
                            elif bin_num == 3:
                                str_buf3.append(data)
                            #print('str_buf:'+str(list(map(hex,str_buf))))
                        for hex in line[4:-1]:
                            if bin_num == 1:
                                bin_buf1.append(hex)
                            if bin_num == 2:
                                bin_buf2.append(hex)
                            if bin_num == 3:
                                bin_buf3.append(hex)
                        if LL!=0x10: #if LL!=0x10:
                            addr_end = addr_h + addr_l + LL - 1
                            addr_start = addr_end + 1 - byte_num
                            print(' *Addr_start: 0x%08X' %addr_start)
                            print(' *Addr_end  : 0x%08X' %addr_end)
                            print(' *Size : %d Bytes' %byte_num)
                        else: 
                            pass
                    else:
                        print('checksum failed!'+str(list(map(hex,line))))
                elif(line[7:9] == '05'):
                    #print('Extended Segment Address Record');
                    line = char2hex(line)
                    if checksum(line) == 0:    pass
                    else:
                        print('checksum failed!'+str(list(map(hex,line))))
                elif(line[7:9] == '01'):         #End of FileRecord
                    #print('End of FileRecord');
                    line = char2hex(line)
                    if checksum(line) == 0:
                        print(" ------------------------------")
                        print(' *Hexf file successed resolved*')
                        print(' *Total bin files : %d ' %bin_num)
                        if bin_num == 1:
                            write_str(str_buf1, 1, addr_start)
                            bin1_chksum = reduce(lambda x,y:x+y,bin_buf1[:])
                            bin_info = "const char bin1_chksum[] = \"%x"%bin1_chksum + "\";\n"
                        elif bin_num == 2:
                            write_str(str_buf2, 2, addr_start)
                            bin2_chksum = reduce(lambda x,y:x+y,bin_buf2[:])
                            bin_info += "const char bin2_chksum[] = \"%x"%bin2_chksum + "\";\n"
                        elif bin_num == 3:
                            write_str(str_buf3, 3, addr_start)
                            bin3_chksum = reduce(lambda x,y:x+y,bin_buf3[:])
                            bin_info += "const char bin3_chksum[] = \"%x"%bin3_chksum + "\";\n"
                        bin_info += "const uint8_t bin_total_num = %d"%bin_num + ";\n"
                        imageFile.write(bin_info)
                    else:
                        print('checksum failed!'+str(list(map(hex,line))))
                else:
                    pass       #don't care
            else:
                print('illegal format!')

image_comment = ("/*"
                + "\n* This file was automatically generated."
                + "\n* The bin data is used to update QMS7926."
                + "\n* Created by: QST AE team"
                + "\n*/\n"
                + "\n__weak const char bin1_cpbin_cmd[];"
                + "\n__weak const char bin2_cpbin_cmd[];"
                + "\n__weak const char bin3_cpbin_cmd[];"
                + "\n__weak const char bin1_chksum[];"
                + "\n__weak const char bin2_chksum[];"
                + "\n__weak const char bin3_chksum[];"
                + "\n__weak const uint8_t bin1[];"
                + "\n__weak const uint8_t bin2[];"
                + "\n__weak const uint8_t bin3[];"
                + "\n__weak const uint32_t bin1_char_len;"
                + "\n__weak const uint32_t bin2_char_len;"
                + "\n__weak const uint32_t bin3_char_len;\n")

# write data in str_buf to '*.h' file
def write_str(str_buf, bin_index, address):
    eprom_begin = ("//Convert hexf file part%d"%bin_index + " to array bin%d.\n"%bin_index)
    eprom_end = "};\n"
    eprom_content = ""
    bin_size = int(len(str_buf) / 2)
    cpbin_cmd_payload = "%06x "%(address) + "%x "%(bin_size) + "%08x"%(address)
    for i in range(0, bin_size):
        eprom_content += "0x" + str_buf[2*i] + str_buf[2*i + 1] + ","
        if i % 16 == 15:
            eprom_content += "\n"
    if bin_index == 1:
        eprom_begin += "\nconst uint8_t bin1[] ={\n"
        eprom_end += "\nconst uint32_t bin1_char_len = %d"%(bin_size) + ";\n"
        eprom_end += "\nconst char bin1_cpbin_cmd[] = \"cpbin c0 " + cpbin_cmd_payload + "\";\n"
    elif bin_index == 2:
        eprom_begin += "\nconst uint8_t bin2[] ={\n"
        eprom_end += "\nconst uint32_t bin2_char_len = %d"%(bin_size) + ";\n"
        eprom_end += "\nconst char bin2_cpbin_cmd[] = \"cpbin c1 " + cpbin_cmd_payload + "\";\n"
    elif bin_index == 3:
        eprom_begin += "\nconst uint8_t bin3[] ={\n"
        eprom_end += "\nconst uint32_t bin3_char_len = %d"%(bin_size) + ";\n"
        eprom_end += "\nconst char bin3_cpbin_cmd[] = \"cpbin c2 " + cpbin_cmd_payload + "\";\n"
    imageFile.write("\n" + eprom_begin + eprom_content + eprom_end + "\n")

#one line string to hex-8 list,except ':' and CR
def char2hex(line):
    line=list(map(ord,list(line)))
    for num in range(len(line)):
        if line[num]>=0x30 and line[num]<=0x39:
            line[num] = line[num] - 0x30
        elif line[num]>=0x41 and line[num]<=0x5A:
            line[num] = line[num] - 55
        else:
            pass
    line=line[1:-1]     #delete CR and ':', in terms of byte
    for i in range(0,len(line),2):
        line[i] = line[i]*16 + line[i+1]
        newline = line[::2]
    return newline
    
#checksum calculation of every line
def checksum(line):
    #considering if the checksum calculation result is 0x100
    sum = (0x100 - (reduce(lambda x,y:x+y,line[:-1]) % 256)) % 256
    if sum == line[-1]:     #check if sum calculated is equal to checksum byte in hex file
        return 0
    else:
        return 1

starttime = time.perf_counter()
hex_file_name = getFileNamebyEX('.')
#h_file_name = hex_file_name[:-4]+'.h'
h_file_name = hex_file_name[:-5]+'.h'
imageFile = open(h_file_name, "w")
imageFile.write(image_comment)
hex2str(hex_file_name,h_file_name)
imageFile.close()
endtime = time.perf_counter()
  
print(' *Time elapsed:' + str(endtime-starttime))