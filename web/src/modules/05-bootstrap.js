/* ==================================================================
   05-bootstrap.js —— 收尾：把默认配置装上

   ★ 必须排在【所有模块与配置文件之后】执行。
     原因：applyConfig() 要写 MODEL / RULES / MAP / CHASSIS / SIZER，
     这些变量的 let 声明在 01 / 02 / 03 里。它们在同一个 <script> 作用域中，
     而 let 有暂时性死区（TDZ）—— 在本文件位置之前访问会抛 ReferenceError。
     放在最后，声明都已执行完，就可以安全赋值。

   构建顺序（见 web/lib/assemble.js）：
       01-kinematics → 02-task-rules → 03-chassis-size → 04-config
       → configs/*.js（按 index.txt）→ 05-bootstrap
   ==================================================================================== */
'use strict';

initDefaultConfig();
