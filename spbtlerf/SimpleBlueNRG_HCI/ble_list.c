/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : ble_list.c
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 19-July-2012
* Description        : Circular Linked List Implementation.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************
 * Include Files
******************************************************************************/
#include <hal_types.h>
#include "ble_list.h"
#include "irq.h"

/******************************************************************************
 * Function Definitions
******************************************************************************/
void ble_list_init_head (tListNode * listHead)
{
  listHead->next = listHead;
  listHead->prev = listHead;
}

uint8_t ble_list_is_empty (tListNode * listHead)
{
  uint32_t uwPRIMASK_Bit;
  uint8_t return_value;

  uwPRIMASK_Bit = irq_disable();

  if(listHead->next == listHead)
  {
    return_value = TRUE;
  }
  else
  {
    return_value = FALSE;
  }
  irq_restore(uwPRIMASK_Bit);

  return return_value;
}

void ble_list_insert_head (tListNode * listHead, tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  node->next = listHead->next;
  node->prev = listHead;
  listHead->next = node;
  (node->next)->prev = node;

  irq_restore(uwPRIMASK_Bit);
}

void ble_list_insert_tail (tListNode * listHead, tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  node->next = listHead;
  node->prev = listHead->prev;
  listHead->prev = node;
  (node->prev)->next = node;

  irq_restore(uwPRIMASK_Bit);
}

void ble_list_remove_node (tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  (node->prev)->next = node->next;
  (node->next)->prev = node->prev;

  irq_restore(uwPRIMASK_Bit);
}

void ble_list_remove_head (tListNode * listHead, tListNode ** node )
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  *node = listHead->next;
  ble_list_remove_node (listHead->next);
  (*node)->next = NULL;
  (*node)->prev = NULL;

  irq_restore(uwPRIMASK_Bit);
}

void ble_list_remove_tail (tListNode * listHead, tListNode ** node )
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  *node = listHead->prev;
  ble_list_remove_node (listHead->prev);
  (*node)->next = NULL;
  (*node)->prev = NULL;

  irq_restore(uwPRIMASK_Bit);
}

void ble_list_insert_node_after (tListNode * node, tListNode * ref_node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  node->next = ref_node->next;
  node->prev = ref_node;
  ref_node->next = node;
  (node->next)->prev = node;

  irq_restore(uwPRIMASK_Bit);
}

void ble_list_insert_node_before (tListNode * node, tListNode * ref_node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  node->next = ref_node;
  node->prev = ref_node->prev;
  ref_node->prev = node;
  (node->prev)->next = node;

  irq_restore(uwPRIMASK_Bit);
}

int ble_list_get_size (tListNode * listHead)
{
  int size = 0;
  tListNode * temp;
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  temp = listHead->next;
  while (temp != listHead)
  {
    size++;
    temp = temp->next;
  }

  irq_restore(uwPRIMASK_Bit);
  return (size);
}

void ble_list_get_next_node (tListNode * ref_node, tListNode ** node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  *node = ref_node->next;

  irq_restore(uwPRIMASK_Bit);
}

void ble_list_get_prev_node (tListNode * ref_node, tListNode ** node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = irq_disable();

  *node = ref_node->prev;

  irq_restore(uwPRIMASK_Bit);
}


#ifdef __cplusplus
 }
#endif
