// Проверка логики автономного HTML без браузера и соединения с роботом.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict'),path=require('node:path');
const file=path.resolve(__dirname,'../../outputs/go2-autonomy/virtual-arena/Карта и виртуальный прогон.html');
const html=fs.readFileSync(file,'utf8');
const data=html.match(/<script id="data" type="application\/json">([\s\S]*?)<\/script>/)[1];
const code=html.match(/<\/script><script>([\s\S]*?)<\/script>/)[1];
const elements=new Map();
const canvas=new Proxy({}, {get:(o,k)=>o[k]??((...args)=>{for(const a of args)if(typeof a==='number')assert.ok(Number.isFinite(a),'Неконечная координата '+k)}),set:(o,k,v)=>(o[k]=v,true)});
function element(id){if(!elements.has(id))elements.set(id,{value:id==='speed'?'0.25':'',checked:id==='showMargin',textContent:id==='data'?data:'',children:[],append(...x){this.children.push(...x)},replaceChildren(){this.children=[]},removeAttribute(){},getContext(){return canvas},getBoundingClientRect(){return{width:800,height:570,left:0,top:0}}});return elements.get(id)}
const ctx=vm.createContext({console,document:{getElementById:element,createElement:()=>({append(){}}),createTextNode:t=>t},devicePixelRatio:1,window:{addEventListener(){}},requestAnimationFrame(){},setInterval(){return 1},clearInterval(){}});
vm.runInContext(code,ctx);
vm.runInContext(`
for(ds=0;ds<DATA.datasets.length;ds++)for(fi=0;fi<dataset().frames.length;fi++)load();
ds=4;fi=0;load();
if(!route().length)throw Error('Нет учебного пути');
draw=()=>{};
$('run').onclick();
for(let t=100;t<200000;t+=100)animate(t);
if(playing||distance!==routeLength(route()))throw Error('Прогон не завершился');
$('reset').onclick();
obstacles=[atDistance(route(),.3)];
$('run').onclick();animate(200100);
if(playing||distance!==0)throw Error('Не остановился перед препятствием');
$('clear').onclick();$('run').onclick();animate(200200);
if(!playing||distance<=0)throw Error('Не продолжил после очистки');
$('stop').onclick();if(playing)throw Error('Не сработала пауза');
ds=0;fi=1;load();if(!$('run').disabled)throw Error('Разрешён прогон неподтверждённой записи');
`,ctx);
assert.equal(elements.get('dataset').children.length,5);
console.log('OK: 13 сцен, конечные координаты, полный учебный путь, остановка/продолжение, пауза, запрет прогона записей. Визуальная проверка браузером не выполнена.');

