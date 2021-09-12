#include <linux/list.h>
#include <linux/spinlock.h>
#include "sn_ether.h"

static LIST_HEAD(ysn3_client_list);
static DEFINE_MUTEX(ysn3_common_lock);

extern  struct dummy_device roce_dummy_device;

int ysn3_register_client(struct ysn3_client *client)
{
	struct ysn3_client *client_tmp;
	struct ysn3_handle *roce;
	int ret = 0;

	pr_info("%s: client register, type:%d\n", __func__, client->type);

	mutex_lock(&ysn3_common_lock);
	/* one system should only have one client for every type */
	list_for_each_entry(client_tmp, &ysn3_client_list, node) {
		if (client_tmp->type == client->type)
			goto exit;
	}

    list_add_tail(&client->node, &ysn3_client_list);

    //Just call client directly by using dummy roce, just for testing
    roce = &roce_dummy_device.roce;
    roce->pdev = roce_dummy_device.dev;

	ret = client->ops->init_instance(roce);
	if (ret) {
		pr_err("Init roce client instance failed: %d\n", client->type);
			goto exit;
	}

exit:
	mutex_unlock(&ysn3_common_lock);

	return 0;
}
EXPORT_SYMBOL(ysn3_register_client);

void ysn3_unregister_client(struct ysn3_client *client)
{
	struct ysn3_handle *roce;

	pr_info("%s: client unregister, type:%d\n", __func__, client->type);

	mutex_lock(&ysn3_common_lock);
	//Just call client directly by using dummy roce, just for testing
    roce = &roce_dummy_device.roce;
    roce->pdev = roce_dummy_device.dev;
	client->ops->uninit_instance(roce, 0);

	list_del(&client->node);

	mutex_unlock(&ysn3_common_lock);
}
EXPORT_SYMBOL(ysn3_unregister_client);