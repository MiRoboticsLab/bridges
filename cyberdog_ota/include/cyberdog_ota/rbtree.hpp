#ifndef	_LINUX_RBTREE_H
#define	_LINUX_RBTREE_H

#pragma pack(1)
struct rb_node
{
	struct rb_node *rb_parent;
	struct rb_node *rb_right;
	struct rb_node *rb_left;
	char rb_color;
#define	RB_RED		0
#define	RB_BLACK	1
};
#pragma pack()

struct rb_root
{
	struct rb_node *rb_node;
};

#define RB_ROOT (struct rb_root){ (struct rb_node *)0, }
#define	rb_entry(ptr, type, member) \
	((type *)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

#ifdef __cplusplus
extern "C"
{
#endif

extern void rb_insert_color(struct rb_node *node, struct rb_root *root);
extern void rb_erase(struct rb_node *node, struct rb_root *root);

/* Find logical next and previous nodes in a tree */
extern struct rb_node *rb_next(struct rb_node *);
extern struct rb_node *rb_prev(struct rb_node *);
extern struct rb_node *rb_first(struct rb_root *);
extern struct rb_node *rb_last(struct rb_root *);

/* Fast replacement of a single node without remove/rebalance/add/rebalance */
extern void rb_replace_node(struct rb_node *victim, struct rb_node *newnode, 
			    struct rb_root *root);

#ifdef __cplusplus
}
#endif

static inline void rb_link_node(struct rb_node *node, struct rb_node *parent,
				struct rb_node **link)
{
	node->rb_parent = parent;
	node->rb_color = RB_RED;
	node->rb_left = node->rb_right = (struct rb_node *)0;

	*link = node;
}

#endif
