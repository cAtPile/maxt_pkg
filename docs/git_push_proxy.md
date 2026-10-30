# Git push 无法解析 GitHub 的处理方法

如果推送时报错：

```text
ssh: Could not resolve hostname github.com: Temporary failure in name resolution
```

说明本机 DNS 无法解析 GitHub。本次环境中 HTTP/HTTPS 代理可用，但 Git 使用的 SSH 默认不会读取 `HTTP_PROXY`、`HTTPS_PROXY` 或 `ALL_PROXY`，因此需要单独指定代理。

## 1. 检查代理是否可用

```bash
printenv | rg -i '^(https?_proxy|all_proxy)='
curl -I --connect-timeout 10 https://github.com
```

本次代理地址为 `172.20.144.1:7890`。如果代理地址变化，请替换下方命令中的地址。`curl` 返回 GitHub 的 HTTP 响应（本次为 `200`）说明代理链路可用。

## 2. 通过代理推送

确保代理程序已启动，并且系统已安装支持 `-X connect` 的 `nc`（本次使用 OpenBSD netcat）。执行：

```bash
git -c 'core.sshCommand=ssh -o ProxyCommand="nc -X connect -x 172.20.144.1:7890 %h %p" -o ConnectTimeout=15 -o BatchMode=yes' push
```

此命令让 SSH 通过 HTTP CONNECT 代理连接 GitHub，并继续使用已有 SSH 密钥。配置只对本次命令生效。当前分支需要已设置上游；若没有，可在命令末尾使用 `push -u origin 分支名` 替换 `push`。

## 3. 确认结果

```bash
git status --short --branch
```

推送成功后，当前分支不再显示 `ahead`。如果出现 `Permission denied (publickey)`，则需要检查 SSH 密钥和仓库权限；这是另一个问题。
