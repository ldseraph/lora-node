#include <lorawan.h>
#define DBG_SECTION_NAME "lorawan"
#include <ulog.h>

static rt_size_t lorawan_serializer_mhdr(uint8_t* buffer, lorawan_ftype_t ftype) {
  lorawan_mhdr_t mhdr = {
    .bits = {
      .major = LORAWAN_MAJOR_R1,
      .rfu   = LORAWAN_RFU,
      .ftype = ftype,
    }
  };
  *buffer = mhdr.value;
  return sizeof(lorawan_mhdr_t);
}

rt_err_t lorawan_serializer_compute_hash(
  uint8_t* bx_buffer,
  uint8_t* buffer,
  uint16_t size,
  uint8_t* key,
  uint8_t* hash) {
  uint8_t              c_mac[16]    = { 0 };
  lorawan_aes_cmac_ctx aes_cmac_ctx = { 0 };

  lorawan_cmac_setkey(&aes_cmac_ctx, key);

  if (bx_buffer != NULL) {
    lorawan_cmac_update(&aes_cmac_ctx, bx_buffer, AES_CMAC_KEY_LENGTH);
  }

  lorawan_cmac_update(&aes_cmac_ctx, buffer, size);

  lorawan_cmac_final(&aes_cmac_ctx, c_mac);

  for (uint32_t i = 0; i < 8; i++) {
    hash[i] = c_mac[i];
  }

  return RT_EOK;
}

static rt_err_t lorawan_serializer_compute_cmac(
  uint8_t*  bx_buffer,
  uint8_t*  buffer,
  uint16_t  size,
  uint8_t*  key,
  uint32_t* cmac) {
  uint8_t              c_mac[16]    = { 0 };
  lorawan_aes_cmac_ctx aes_cmac_ctx = { 0 };

  lorawan_cmac_setkey(&aes_cmac_ctx, key);

  if (bx_buffer != NULL) {
    lorawan_cmac_update(&aes_cmac_ctx, bx_buffer, AES_CMAC_KEY_LENGTH);
  }

  lorawan_cmac_update(&aes_cmac_ctx, buffer, size);

  lorawan_cmac_final(&aes_cmac_ctx, c_mac);

  *cmac = (uint32_t)((uint32_t)c_mac[3] << 24 | (uint32_t)c_mac[2] << 16 | (uint32_t)c_mac[1] << 8 | (uint32_t)c_mac[0]);

  return RT_EOK;
}

static rt_err_t lorawan_serializer_aes_encrypt(
  uint8_t* in_buffer,
  uint16_t size,
  uint8_t* key,
  uint8_t* out_buffer) {
  lorawan_aes_cmac_ctx aes_cmac_ctx = { 0 };
  if ((size % N_BLOCK) != 0) {
    return -RT_EINVAL;
  }

  lorawan_soft_se_aes_set_key(&aes_cmac_ctx.rijndael, key, N_BLOCK);
  uint8_t block = 0;

  while (size != 0) {
    lorawan_soft_se_aes_encrypt(&in_buffer[block], &out_buffer[block], &aes_cmac_ctx.rijndael);
    block = block + N_BLOCK;
    size  = size - N_BLOCK;
  }

  return RT_EOK;
}

static void lorawan_crypto_derive_mc_root_key(lorawan_t* lorawan, uint32_t version) {
  uint8_t compBase[16] = { 0 };

  if (version == 1) {
    compBase[0] = 0x20;
  }

  lorawan_soft_se_derive(lorawan->secure_element.app_key, compBase, lorawan->secure_element.mc_root_key);
}

static void lorawan_crypto_derive_mc_ke_key(lorawan_t* lorawan) {
  uint8_t comp_base[16] = { 0 };
  lorawan_soft_se_derive(lorawan->secure_element.mc_root_key, comp_base, lorawan->secure_element.mc_ke_key);
}

static void lorawan_crypto_derive_session_key_10x(
  lorawan_t* lorawan,
  uint32_t   key_comp,
  uint8_t*   out_key) {
  uint8_t comp_base[16] = {
    [0]        = key_comp,
    [1]        = (uint8_t)((lorawan->join_nonce >> 0) & 0xFF),
    [2]        = (uint8_t)((lorawan->join_nonce >> 8) & 0xFF),
    [3]        = (uint8_t)((lorawan->join_nonce >> 16) & 0xFF),
    [4]        = (uint8_t)((lorawan->net_ID >> 0) & 0xFF),
    [5]        = (uint8_t)((lorawan->net_ID >> 8) & 0xFF),
    [6]        = (uint8_t)((lorawan->net_ID >> 16) & 0xFF),
    [7]        = (uint8_t)((lorawan->dev_nonce >> 0) & 0xFF),
    [8]        = (uint8_t)((lorawan->dev_nonce >> 8) & 0xFF),
    [9 ... 15] = 0,
  };

  lorawan_soft_se_derive(lorawan->secure_element.app_key, comp_base, out_key);

  return;
}

static void memcpyr(uint8_t* dst, const uint8_t* src, uint32_t size) {
  dst = dst + (size - 1);
  while (size--) {
    *dst-- = *src++;
  }
}

lorawan_ftype_t lorawan_serializer_get_ftype(uint8_t value) {
  lorawan_mhdr_t mhdr = {
    .value = value,
  };
  return mhdr.bits.ftype;
}

rt_err_t lorawan_serializer_join_request(lorawan_t* lorawan, uint8_t* buffer) {
  // mhdr
  rt_size_t size = lorawan_serializer_mhdr(buffer, LORAWAN_FTYPE_JOIN_REQUEST);

  // join eui
  memcpyr(&buffer[size], lorawan->secure_element.join_eui, LORAWAN_JOINEUI_SIZE);
  size += LORAWAN_JOINEUI_SIZE;

  // dev eui
  memcpyr(&buffer[size], lorawan->secure_element.dev_eui, LORAWAN_DEVEUI_SIZE);
  size += LORAWAN_DEVEUI_SIZE;

  // devnonce
  uint32_t dev_nonce;
  rt_err_t err = lora_radio_random(lorawan->lora_radio_device, &dev_nonce);
  if (err != RT_EOK) {
    LOG_E("serializer join request nonce err: %d.", err);
    return err;
  }

  lorawan->dev_nonce = (uint16_t)dev_nonce;
  rt_memcpy(&buffer[size], (const uint8_t*)&lorawan->dev_nonce, LORAWAN_DEVNONCE_SIZE);
  size += LORAWAN_DEVNONCE_SIZE;

  // mic
  uint32_t mic;
  err = lorawan_serializer_compute_cmac(RT_NULL, buffer, size, lorawan->secure_element.app_key, &mic);
  if (err != RT_EOK) {
    LOG_E("serializer join request cmac err: %d.", err);
    return err;
  }

  buffer[size++] = mic & 0xFF;
  buffer[size++] = (mic >> 8) & 0xFF;
  buffer[size++] = (mic >> 16) & 0xFF;
  buffer[size++] = (mic >> 24) & 0xFF;

  return RT_EOK;
}

rt_err_t lorawan_serializer_join_accept(
  lorawan_t* lorawan,
  uint8_t*   buffer,
  rt_size_t  size) {
  uint8_t dec[LORAMAC_JOIN_ACCEPT_FRAME_MAX_SIZE] = { 0 };

  if (size > LORAMAC_JOIN_ACCEPT_FRAME_MAX_SIZE) {
    LOG_E("serializer join accept size(%d) err.", size);
    return -RT_EINVAL;
  }

  rt_memcpy(dec, buffer, LORAMAC_MHDR_FIELD_SIZE);

  rt_err_t err = lorawan_serializer_aes_encrypt(
    buffer + LORAMAC_MHDR_FIELD_SIZE,
    size - LORAMAC_MHDR_FIELD_SIZE,
    lorawan->secure_element.app_key,
    dec + LORAMAC_MHDR_FIELD_SIZE);
  if (err != RT_EOK) {
    LOG_E("serializer join accept aes encrypt err:%d", err);
    return err;
  }

  uint8_t version = ((dec[11] & 0x80) == 0x80) ? 1 : 0;

  uint32_t mic = 0;

  mic = ((uint32_t)dec[size - LORAMAC_MIC_FIELD_SIZE] << 0);
  mic |= ((uint32_t)dec[size - LORAMAC_MIC_FIELD_SIZE + 1] << 8);
  mic |= ((uint32_t)dec[size - LORAMAC_MIC_FIELD_SIZE + 2] << 16);
  mic |= ((uint32_t)dec[size - LORAMAC_MIC_FIELD_SIZE + 3] << 24);

  if (version != 0) {
    LOG_E("serializer join accept version(%d) err.", version);
    return -RT_ERROR;
  }

  uint32_t verify_mic;
  err = lorawan_serializer_compute_cmac(RT_NULL, dec, size - LORAMAC_MIC_FIELD_SIZE, lorawan->secure_element.app_key, &verify_mic);
  if (err != RT_EOK) {
    LOG_E("serializer join accept cmac err: %d.", err);
    return err;
  }

  if (mic != verify_mic) {
    LOG_E("serializer join accept verify mic err: mic(%x) verify_mic(%x).", mic, verify_mic);
    return -RT_ERROR;
  }

  uint32_t index = 0;
  index++;
  uint32_t current_join_nonce = (uint32_t)dec[index]
                                | ((uint32_t)dec[index + 1] << 8)
                                | ((uint32_t)dec[index + 2] << 16);
  index += LORAMAC_JOIN_NONCE_FIELD_SIZE;

  if (current_join_nonce == lorawan->join_nonce) {
    LOG_E("serializer join accept join nonce(0x%x) err.", current_join_nonce);
    return -RT_EINVAL;
  }
  lorawan->join_nonce = current_join_nonce;

  lorawan->net_ID = (uint32_t)dec[index]
                    | ((uint32_t)dec[index + 1] << 8)
                    | ((uint32_t)dec[index + 2] << 16);
  index += LORAMAC_NET_ID_FIELD_SIZE;

  lorawan->dev_addr = (uint32_t)dec[index]
                      | ((uint32_t)dec[index + 1] << 8)
                      | ((uint32_t)dec[index + 2] << 16)
                      | ((uint32_t)dec[index + 3] << 24);
  index += LORAMAC_DEV_ADDR_FIELD_SIZE;

  lorawan_crypto_derive_mc_root_key(lorawan, version);
  lorawan_crypto_derive_mc_ke_key(lorawan);
  lorawan_crypto_derive_session_key_10x(lorawan, LORAWAN_APP_S_KEY_COMP_BASE, lorawan->secure_element.app_s_key);
  lorawan_crypto_derive_session_key_10x(lorawan, LORAWAN_NWK_S_KEY_COMP_BASE, lorawan->secure_element.nwk_s_enc_key);
  lorawan_crypto_derive_session_key_10x(lorawan, LORAWAN_NWK_S_KEY_COMP_BASE, lorawan->secure_element.f_nwk_s_int_key);
  lorawan_crypto_derive_session_key_10x(lorawan, LORAWAN_NWK_S_KEY_COMP_BASE, lorawan->secure_element.s_nwk_s_int_key);

  lorawan_dl_setting_t dl_settings = {
    .value = dec[index++],
  };
  lorawan->rx1_dr_offset = dl_settings.bits.rx1_dr_offset;
  lorawan->rx2_dr        = dl_settings.bits.rx2_datarate;
  lorawan->rxc_dr        = dl_settings.bits.rx2_datarate;

  uint32_t rx_delay = dec[index++];
  if (rx_delay == 0) {
    rx_delay = 1;
  }
  lorawan->receive_delay1 = rx_delay * 1000;
  lorawan->receive_delay2 = lorawan->receive_delay1 + 1000;

  lorawan->version.fields.major = version;

  lorawan->fcnt_list.fcnt_up     = 0;
  lorawan->fcnt_list.fcnt_down   = LORAMAC_FCNT_DOWN_INITAL_VALUE;
  lorawan->fcnt_list.n_fcnt_down = LORAMAC_FCNT_DOWN_INITAL_VALUE;
  lorawan->fcnt_list.a_fcnt_down = LORAMAC_FCNT_DOWN_INITAL_VALUE;

  if ((size - LORAMAC_MIC_FIELD_SIZE - index) == LORAMAC_CF_LIST_FIELD_SIZE) {
    uint8_t cf_list[LORAMAC_CF_LIST_FIELD_SIZE];

    rt_memcpy(cf_list, &dec[index], LORAMAC_CF_LIST_FIELD_SIZE);

    rt_err_t err = lorawan->region->apply_cf_list(lorawan, cf_list);
    if (err != RT_EOK) {
      LOG_E("region apply cf list err:%d", err);
      return RT_NULL;
    }
    index += LORAMAC_CF_LIST_FIELD_SIZE;
  } else if ((size - LORAMAC_MIC_FIELD_SIZE - index) > 0) {
    LOG_E("serializer join accept cf size(%d) err.", (size - LORAMAC_MIC_FIELD_SIZE - index));
    return -RT_EINVAL;
  }

  return RT_EOK;
}

typedef enum {
  UPLINK   = 0,
  DOWNLINK = 1
} lorawan_payload_encrypt_dir_t;

static rt_err_t lorawan_serializer_payload_encrypt(
  lorawan_t*                    lorawan,
  int16_t                       size,
  lorawan_payload_encrypt_dir_t dir,
  uint32_t                      fcnt,
  uint8_t*                      key,
  uint8_t*                      buffer) {
  uint8_t  index      = 0;
  uint16_t ctr        = 1;
  uint8_t  sBlock[16] = { 0 };
  uint8_t  aBlock[16] = { 0 };

  aBlock[0] = 0x01;

  aBlock[5] = dir;

  aBlock[6] = lorawan->dev_addr & 0xFF;
  aBlock[7] = (lorawan->dev_addr >> 8) & 0xFF;
  aBlock[8] = (lorawan->dev_addr >> 16) & 0xFF;
  aBlock[9] = (lorawan->dev_addr >> 24) & 0xFF;

  aBlock[10] = fcnt & 0xFF;
  aBlock[11] = (fcnt >> 8) & 0xFF;
  aBlock[12] = (fcnt >> 16) & 0xFF;
  aBlock[13] = (fcnt >> 24) & 0xFF;

  while (size > 0) {
    aBlock[15] = ctr & 0xFF;
    ctr++;

    rt_err_t err = lorawan_serializer_aes_encrypt(
      aBlock,
      sizeof(aBlock),
      key,
      sBlock);
    if (err != RT_EOK) {
      LOG_E("serializer payload encrypt err:%d", err);
      return err;
    }

    for (uint8_t i = 0; i < ((size > 16) ? 16 : size); i++) {
      buffer[index + i] = buffer[index + i] ^ sBlock[i];
    }
    size -= 16;
    index += 16;
  }

  return RT_EOK;
}

#define CRYPTO_MAXMESSAGE_SIZE 256
#define MIC_BLOCK_BX_SIZE 16
#define CRYPTO_BUFFER_SIZE CRYPTO_MAXMESSAGE_SIZE + MIC_BLOCK_BX_SIZE

static rt_err_t lorawan_serializer_compute_cmac_B0(
  lorawan_t*                    lorawan,
  lorawan_tx_msg_t*             msg,
  uint32_t                      size,
  lorawan_payload_encrypt_dir_t dir,
  uint8_t*                      key,
  uint32_t*                     mic) {
  uint8_t b0[MIC_BLOCK_BX_SIZE];
  b0[0]  = 0x49;
  b0[1]  = 0x00;
  b0[2]  = 0x00;
  b0[3]  = 0x00;
  b0[4]  = 0x00;
  b0[5]  = dir;
  b0[6]  = lorawan->dev_addr & 0xFF;
  b0[7]  = (lorawan->dev_addr >> 8) & 0xFF;
  b0[8]  = (lorawan->dev_addr >> 16) & 0xFF;
  b0[9]  = (lorawan->dev_addr >> 24) & 0xFF;
  b0[10] = msg->fcnt_up & 0xFF;
  b0[11] = (msg->fcnt_up >> 8) & 0xFF;
  b0[12] = (msg->fcnt_up >> 16) & 0xFF;
  b0[13] = (msg->fcnt_up >> 24) & 0xFF;
  b0[14] = 0x00;
  b0[15] = size & 0xFF;

  rt_err_t err = lorawan_serializer_compute_cmac(
    b0,
    msg->phy_msg.buffer,
    size,
    key,
    mic);
  if (err != RT_EOK) {
    LOG_E("compute cmac err: %d.", err);
    return err;
  }

  return RT_EOK;
}

static rt_err_t lorawan_serializer_verify_cmac_B0(
  lorawan_t*                    lorawan,
  lorawan_rx_msg_t*             msg,
  uint32_t                      size,
  lorawan_payload_encrypt_dir_t dir) {
  uint8_t b0[CRYPTO_BUFFER_SIZE];
  b0[0]  = 0x49;
  b0[1]  = 0x00;
  b0[2]  = 0x00;
  b0[3]  = 0x00;
  b0[4]  = 0x00;
  b0[5]  = dir;
  b0[6]  = msg->dev_addr & 0xFF;
  b0[7]  = (msg->dev_addr >> 8) & 0xFF;
  b0[8]  = (msg->dev_addr >> 16) & 0xFF;
  b0[9]  = (msg->dev_addr >> 24) & 0xFF;
  b0[10] = msg->fcnt_down & 0xFF;
  b0[11] = (msg->fcnt_down >> 8) & 0xFF;
  b0[12] = (msg->fcnt_down >> 16) & 0xFF;
  b0[13] = (msg->fcnt_down >> 24) & 0xFF;
  b0[14] = 0x00;
  b0[15] = size & 0xFF;

  rt_memcpy(b0 + MIC_BLOCK_BX_SIZE, msg->phy_msg.buffer, size);

  uint32_t mic;
  rt_err_t err = lorawan_serializer_compute_cmac(
    RT_NULL,
    b0,
    size + MIC_BLOCK_BX_SIZE,
    lorawan->secure_element.s_nwk_s_int_key,
    &mic);
  if (err != RT_EOK) {
    LOG_E("verify mic compute cmac err: %d.", err);
    return err;
  }

  if (mic != msg->mic) {
    LOG_E("verify mic %x msg->mic %x", mic, msg->mic);
    return -RT_EINVAL;
  }

  return RT_EOK;
}

rt_err_t lorawan_serializer_data(lorawan_t* lorawan, lorawan_tx_msg_t* msg) {
  uint16_t phy_size = LORAMAC_MHDR_FIELD_SIZE
                      + LORAMAC_FHDR_DEV_ADDR_FIELD_SIZE
                      + LORAMAC_FHDR_F_CTRL_FIELD_SIZE
                      + LORAMAC_FHDR_F_CNT_FIELD_SIZE;

  phy_size += msg->fctrl.bits.fopts_len;

  if (msg->mac_payload.size > 0) {
    phy_size += LORAMAC_F_PORT_FIELD_SIZE;
  }

  phy_size += msg->mac_payload.size;
  phy_size += LORAMAC_MIC_FIELD_SIZE;

  if (phy_size > LORAMAC_PHY_MAXPAYLOAD) {
    return -RT_ENOMEM;
  }

  rt_size_t size = lorawan_serializer_mhdr(msg->phy_msg.buffer, msg->ftype);

  msg->phy_msg.buffer[size++] = (lorawan->dev_addr) & 0xFF;
  msg->phy_msg.buffer[size++] = (lorawan->dev_addr >> 8) & 0xFF;
  msg->phy_msg.buffer[size++] = (lorawan->dev_addr >> 16) & 0xFF;
  msg->phy_msg.buffer[size++] = (lorawan->dev_addr >> 24) & 0xFF;

  msg->phy_msg.buffer[size++] = msg->fctrl.value;

  msg->phy_msg.buffer[size++] = msg->fcnt_up & 0xFF;
  msg->phy_msg.buffer[size++] = (msg->fcnt_up >> 8) & 0xFF;

  rt_memcpy(&msg->phy_msg.buffer[size], msg->fopts, msg->fctrl.bits.fopts_len);
  size += msg->fctrl.bits.fopts_len;

  if (msg->mac_payload.size > 0) {
    msg->phy_msg.buffer[size++] = msg->fport;
  }

  msg->key = lorawan->secure_element.app_s_key;
  if (msg->fport == 0) {
    msg->key = lorawan->secure_element.nwk_s_enc_key;
  }

  rt_err_t err = lorawan_serializer_payload_encrypt(
    lorawan,
    msg->mac_payload.size,
    UPLINK,
    msg->fcnt_up,
    msg->key,
    msg->mac_payload.buffer);
  if (err != RT_EOK) {
    return err;
  }

  rt_memcpy(&msg->phy_msg.buffer[size], msg->mac_payload.buffer, msg->mac_payload.size);
  size += msg->mac_payload.size;

  uint32_t mic;
  err = lorawan_serializer_compute_cmac_B0(
    lorawan,
    msg,
    size,
    UPLINK,
    lorawan->secure_element.nwk_s_enc_key,
    &mic);
  if (err != RT_EOK) {
    LOG_E("serializer join request cmac err: %d.", err);
    return err;
  }

  msg->phy_msg.buffer[size++] = mic & 0xFF;
  msg->phy_msg.buffer[size++] = (mic >> 8) & 0xFF;
  msg->phy_msg.buffer[size++] = (mic >> 16) & 0xFF;
  msg->phy_msg.buffer[size++] = (mic >> 24) & 0xFF;

  msg->phy_msg.size = size;

  return RT_EOK;
}

rt_err_t lorawan_deserialize_data(lorawan_t* lorawan, lorawan_rx_msg_t* msg) {
  uint16_t index = 0;

  msg->ftype = lorawan_serializer_get_ftype(msg->phy_msg.buffer[index++]);

  msg->dev_addr = msg->phy_msg.buffer[index++];
  msg->dev_addr |= ((uint32_t)msg->phy_msg.buffer[index++] << 8);
  msg->dev_addr |= ((uint32_t)msg->phy_msg.buffer[index++] << 16);
  msg->dev_addr |= ((uint32_t)msg->phy_msg.buffer[index++] << 24);

  if (msg->dev_addr != lorawan->dev_addr) {
    LOG_E("lorawan addr(%x) msg addr(%x) err", lorawan->dev_addr, msg->dev_addr);
    return -RT_EINVAL;
  }
  
  msg->fctrl.value = msg->phy_msg.buffer[index++];

  msg->fcnt_down = msg->phy_msg.buffer[index++];
  msg->fcnt_down |= msg->phy_msg.buffer[index++] << 8;

  if (msg->fctrl.bits.fopts_len > 15) {
    return -RT_EINVAL;
  }

  rt_memcpy(msg->fopts, &msg->phy_msg.buffer[index], msg->fctrl.bits.fopts_len);
  index += msg->fctrl.bits.fopts_len;

  msg->fport            = 0;
  msg->mac_payload.size = 0;

  int32_t payload_size = (int32_t)msg->phy_msg.size - index - LORAMAC_MIC_FIELD_SIZE;
  if (payload_size < 0) {
    LOG_E("phy size %d index %d", msg->phy_msg.size, index);
    LOG_E("fopts_len %d fctrl %x", msg->fctrl.bits.fopts_len, msg->fctrl.value);
    return -RT_EINVAL;
  }

  if (payload_size > 0) {
    msg->fport            = msg->phy_msg.buffer[index++];
    msg->mac_payload.size = payload_size;

    rt_memcpy(msg->mac_payload.buffer, &msg->phy_msg.buffer[index], msg->mac_payload.size);
    index += msg->mac_payload.size;
  }

  msg->mic = (uint32_t)msg->phy_msg.buffer[(msg->phy_msg.size - LORAMAC_MIC_FIELD_SIZE)];
  msg->mic |= (uint32_t)msg->phy_msg.buffer[(msg->phy_msg.size - LORAMAC_MIC_FIELD_SIZE) + 1] << 8;
  msg->mic |= (uint32_t)msg->phy_msg.buffer[(msg->phy_msg.size - LORAMAC_MIC_FIELD_SIZE) + 2] << 16;
  msg->mic |= (uint32_t)msg->phy_msg.buffer[(msg->phy_msg.size - LORAMAC_MIC_FIELD_SIZE) + 3] << 24;

  if ((msg->fctrl.bits.fopts_len > 0) && (msg->fport > 0)) {
    msg->frame_type = FRAME_TYPE_A;
  } else if (msg->mac_payload.size == 0) {
    msg->frame_type = FRAME_TYPE_B;
  } else if ((msg->fctrl.bits.fopts_len == 0) && (msg->fport == 0)) {
    msg->frame_type = FRAME_TYPE_C;
  } else if ((msg->fctrl.bits.fopts_len == 0) && (msg->fport > 0)) {
    msg->frame_type = FRAME_TYPE_D;
  } else {
    return -RT_EINVAL;
  }

  msg->key = lorawan->secure_element.app_s_key;
  if (msg->fport == 0) {
    msg->key = lorawan->secure_element.nwk_s_enc_key;
  }

  rt_err_t err = lorawan_serializer_verify_cmac_B0(
    lorawan,
    msg,
    (msg->phy_msg.size - LORAMAC_MIC_FIELD_SIZE),
    DOWNLINK);
  if (err != RT_EOK) {
    return err;
  }

  err = lorawan_serializer_payload_encrypt(
    lorawan,
    msg->mac_payload.size,
    DOWNLINK,
    msg->fcnt_down,
    msg->key,
    msg->mac_payload.buffer);
  if (err != RT_EOK) {
    return err;
  }

  // TODO:Check if it is a multicast message
  int32_t last_down = lorawan->fcnt_list.fcnt_down;
  int32_t fcnt_diff = (int32_t)((int64_t)msg->fcnt_down - (int64_t)(last_down & 0x0000FFFF));
  if (fcnt_diff > 0) {
    lorawan->fcnt_list.fcnt_down = last_down + fcnt_diff;
  } else if (fcnt_diff == 0) {
    // return -RT_EINVAL;
  } else {
    lorawan->fcnt_list.fcnt_down = (last_down & 0xFFFF0000) + 0x10000 + msg->fcnt_down;
  }

  return RT_EOK;
}
