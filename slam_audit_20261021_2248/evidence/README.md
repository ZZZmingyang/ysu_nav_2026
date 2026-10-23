# SLAM audit / evidence 数据

`source_config_assets.tar.gz` 超过 GitHub 的 100 MB 单文件限制，因此拆分为以下三个分卷，每卷不超过 90 MB：

- `source_config_assets.tar.gz.1`
- `source_config_assets.tar.gz.2`
- `source_config_assets.tar.gz.3`

原始文件保留在本地，并由本目录的 `.gitignore` 排除，不提交到 Git 仓库。

在 Linux 下，进入本目录后按顺序合并恢复：

```bash
cat source_config_assets.tar.gz.* > source_config_assets.tar.gz
```

恢复后的 SHA-256：

```text
0145fe61f7e4410332053a0167e4ad7f4af60b86278222289e7c6ab8a46fefce
```

恢复后可以使用以下命令解压：

```bash
tar -xzf source_config_assets.tar.gz
```

这些文件属于临时 SLAM audit / evidence 数据，后续可能从仓库中删除。
