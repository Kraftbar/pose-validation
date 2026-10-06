/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port: a minimal YAML-subset reader (see rd_yaml.c). Nodes are opaque. */
#ifndef RD_YAML_H
#define RD_YAML_H
void* rd_yaml_load(const char* path);                     /* NULL if the file cannot be read */
void rd_yaml_free(void* root);
const void* rd_yaml_find(const void* root, const char* dotted);
int rd_yaml_seq_len(const void* node);                     /* -1 if not a sequence */
const void* rd_yaml_seq_at(const void* node, int i);
const char* rd_yaml_scalar(const void* node);              /* NULL if not a scalar */
#endif
