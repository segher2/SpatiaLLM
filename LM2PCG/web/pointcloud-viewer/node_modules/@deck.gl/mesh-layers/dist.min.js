(function webpackUniversalModuleDefinition(root, factory) {
  if (typeof exports === 'object' && typeof module === 'object')
    module.exports = factory();
  else if (typeof define === 'function' && define.amd) define([], factory);
        else if (typeof exports === 'object') exports['deck'] = factory();
  else root['deck'] = factory();})(globalThis, function () {
"use strict";var __exports__=(()=>{var Ri=Object.create;var et=Object.defineProperty;var Ii=Object.getOwnPropertyDescriptor;var Si=Object.getOwnPropertyNames;var Li=Object.getPrototypeOf,Fi=Object.prototype.hasOwnProperty;var Ut=(t,e)=>()=>(e||t((e={exports:{}}).exports,e),e.exports),N=(t,e)=>{for(var n in e)et(t,n,{get:e[n],enumerable:!0})},$e=(t,e,n,r)=>{if(e&&typeof e=="object"||typeof e=="function")for(let o of Si(e))!Fi.call(t,o)&&o!==n&&et(t,o,{get:()=>e[o],enumerable:!(r=Ii(e,o))||r.enumerable});return t},$=(t,e,n)=>($e(t,e,"default"),n&&$e(n,e,"default")),F=(t,e,n)=>(n=t!=null?Ri(Li(t)):{},$e(e||!t||!t.__esModule?et(n,"default",{value:t,enumerable:!0}):n,t)),Di=t=>$e(et({},"__esModule",{value:!0}),t);var pe=Ut((Zl,rr)=>{rr.exports=globalThis.deck});var oe=Ut((eh,sr)=>{sr.exports=globalThis.luma});var Fe=Ut((th,ir)=>{ir.exports=globalThis.luma});var Ze={};N(Ze,{ScenegraphLayer:()=>Ti,SimpleMeshLayer:()=>Io});var X={},or=F(pe(),1);$(X,F(pe(),1));if(!or.Layer)throw new Error("@deck.gl/core is not found");$(Ze,X);var te=F(pe(),1),Ro=F(oe(),1),ge=F(Fe(),1);var nh=1/Math.PI*180,rh=1/180*Math.PI,vi={EPSILON:1e-12,debug:!1,precision:4,printTypes:!1,printDegrees:!1,printRowMajor:!0,_cartographicRadians:!1};globalThis.mathgl=globalThis.mathgl||{config:{...vi}};var R=globalThis.mathgl.config;function ar(t,{precision:e=R.precision}={}){return t=Gi(t),`${parseFloat(t.toPrecision(e))}`}function me(t){return Array.isArray(t)||ArrayBuffer.isView(t)&&!(t instanceof DataView)}function Ht(t,e,n){let r=R.EPSILON;n&&(R.EPSILON=n);try{if(t===e)return!0;if(me(t)&&me(e)){if(t.length!==e.length)return!1;for(let o=0;o<t.length;++o)if(!Ht(t[o],e[o]))return!1;return!0}return t&&t.equals?t.equals(e):e&&e.equals?e.equals(t):typeof t=="number"&&typeof e=="number"?Math.abs(t-e)<=R.EPSILON*Math.max(1,Math.abs(t),Math.abs(e)):!1}finally{R.EPSILON=r}}function Gi(t){return Math.round(t/R.EPSILON)*R.EPSILON}var ee=class extends Array{clone(){return new this.constructor().copy(this)}fromArray(e,n=0){for(let r=0;r<this.ELEMENTS;++r)this[r]=e[r+n];return this.check()}toArray(e=[],n=0){for(let r=0;r<this.ELEMENTS;++r)e[n+r]=this[r];return e}toObject(e){return e}from(e){return Array.isArray(e)?this.copy(e):this.fromObject(e)}to(e){return e===this?this:me(e)?this.toArray(e):this.toObject(e)}toTarget(e){return e?this.to(e):this}toFloat32Array(){return new Float32Array(this)}toString(){return this.formatString(R)}formatString(e){let n="";for(let r=0;r<this.ELEMENTS;++r)n+=(r>0?", ":"")+ar(this[r],e);return`${e.printTypes?this.constructor.name:""}[${n}]`}equals(e){if(!e||this.length!==e.length)return!1;for(let n=0;n<this.ELEMENTS;++n)if(!Ht(this[n],e[n]))return!1;return!0}exactEquals(e){if(!e||this.length!==e.length)return!1;for(let n=0;n<this.ELEMENTS;++n)if(this[n]!==e[n])return!1;return!0}negate(){for(let e=0;e<this.ELEMENTS;++e)this[e]=-this[e];return this.check()}lerp(e,n,r){if(r===void 0)return this.lerp(this,e,n);for(let o=0;o<this.ELEMENTS;++o){let s=e[o],i=typeof n=="number"?n:n[o];this[o]=s+r*(i-s)}return this.check()}min(e){for(let n=0;n<this.ELEMENTS;++n)this[n]=Math.min(e[n],this[n]);return this.check()}max(e){for(let n=0;n<this.ELEMENTS;++n)this[n]=Math.max(e[n],this[n]);return this.check()}clamp(e,n){for(let r=0;r<this.ELEMENTS;++r)this[r]=Math.min(Math.max(this[r],e[r]),n[r]);return this.check()}add(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]+=n[r];return this.check()}subtract(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]-=n[r];return this.check()}scale(e){if(typeof e=="number")for(let n=0;n<this.ELEMENTS;++n)this[n]*=e;else for(let n=0;n<this.ELEMENTS&&n<e.length;++n)this[n]*=e[n];return this.check()}multiplyByScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]*=e;return this.check()}check(){if(R.debug&&!this.validate())throw new Error(`math.gl: ${this.constructor.name} some fields set to invalid numbers'`);return this}validate(){let e=this.length===this.ELEMENTS;for(let n=0;n<this.ELEMENTS;++n)e=e&&Number.isFinite(this[n]);return e}sub(e){return this.subtract(e)}setScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]=e;return this.check()}addScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]+=e;return this.check()}subScalar(e){return this.addScalar(-e)}multiplyScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]*=e;return this.check()}divideScalar(e){return this.multiplyByScalar(1/e)}clampScalar(e,n){for(let r=0;r<this.ELEMENTS;++r)this[r]=Math.min(Math.max(this[r],e),n);return this.check()}get elements(){return this}};function Oi(t,e){if(t.length!==e)return!1;for(let n=0;n<t.length;++n)if(!Number.isFinite(t[n]))return!1;return!0}function y(t){if(!Number.isFinite(t))throw new Error(`Invalid number ${JSON.stringify(t)}`);return t}function Ae(t,e,n=""){if(R.debug&&!Oi(t,e))throw new Error(`math.gl: ${n} some fields set to invalid numbers'`);return t}function Jt(t,e){if(!t)throw new Error(`math.gl assertion ${e}`)}var tt=class extends ee{get x(){return this[0]}set x(e){this[0]=y(e)}get y(){return this[1]}set y(e){this[1]=y(e)}len(){return Math.sqrt(this.lengthSquared())}magnitude(){return this.len()}lengthSquared(){let e=0;for(let n=0;n<this.ELEMENTS;++n)e+=this[n]*this[n];return e}magnitudeSquared(){return this.lengthSquared()}distance(e){return Math.sqrt(this.distanceSquared(e))}distanceSquared(e){let n=0;for(let r=0;r<this.ELEMENTS;++r){let o=this[r]-e[r];n+=o*o}return y(n)}dot(e){let n=0;for(let r=0;r<this.ELEMENTS;++r)n+=this[r]*e[r];return y(n)}normalize(){let e=this.magnitude();if(e!==0)for(let n=0;n<this.ELEMENTS;++n)this[n]/=e;return this.check()}multiply(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]*=n[r];return this.check()}divide(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]/=n[r];return this.check()}lengthSq(){return this.lengthSquared()}distanceTo(e){return this.distance(e)}distanceToSquared(e){return this.distanceSquared(e)}getComponent(e){return Jt(e>=0&&e<this.ELEMENTS,"index is out of range"),y(this[e])}setComponent(e,n){return Jt(e>=0&&e<this.ELEMENTS,"index is out of range"),this[e]=n,this.check()}addVectors(e,n){return this.copy(e).add(n)}subVectors(e,n){return this.copy(e).subtract(n)}multiplyVectors(e,n){return this.copy(e).multiply(n)}addScaledVector(e,n){return this.add(new this.constructor(e).multiplyScalar(n))}};var S=typeof Float32Array<"u"?Float32Array:Array;var Ah=Math.PI/180;function Pi(){let t=new S(2);return S!=Float32Array&&(t[0]=0,t[1]=0),t}function lr(t,e,n){let r=e[0],o=e[1];return t[0]=n[0]*r+n[4]*o+n[12],t[1]=n[1]*r+n[5]*o+n[13],t}var uh=function(){let t=Pi();return function(e,n,r,o,s,i){let a,c;for(n||(n=2),r||(r=0),o?c=Math.min(o*n+r,e.length):c=e.length,a=r;a<c;a+=n)t[0]=e[a],t[1]=e[a+1],s(t,t,i),e[a]=t[0],e[a+1]=t[1];return e}}();function hr(t,e,n){let r=e[0],o=e[1],s=n[3]*r+n[7]*o||1;return t[0]=(n[0]*r+n[4]*o)/s,t[1]=(n[1]*r+n[5]*o)/s,t}function pr(t,e,n){let r=e[0],o=e[1],s=e[2],i=n[3]*r+n[7]*o+n[11]*s||1;return t[0]=(n[0]*r+n[4]*o+n[8]*s)/i,t[1]=(n[1]*r+n[5]*o+n[9]*s)/i,t[2]=(n[2]*r+n[6]*o+n[10]*s)/i,t}function mr(t,e,n){let r=e[0],o=e[1];return t[0]=n[0]*r+n[2]*o,t[1]=n[1]*r+n[3]*o,t[2]=e[2],t[3]=e[3],t}function Ar(t,e,n){let r=e[0],o=e[1],s=e[2];return t[0]=n[0]*r+n[3]*o+n[6]*s,t[1]=n[1]*r+n[4]*o+n[7]*s,t[2]=n[2]*r+n[5]*o+n[8]*s,t[3]=e[3],t}function Kt(){let t=new S(3);return S!=Float32Array&&(t[0]=0,t[1]=0,t[2]=0),t}function wi(t){let e=t[0],n=t[1],r=t[2];return Math.sqrt(e*e+n*n+r*r)}function Vt(t,e,n){let r=new S(3);return r[0]=t,r[1]=e,r[2]=n,r}function ur(t,e){let n=e[0],r=e[1],o=e[2],s=n*n+r*r+o*o;return s>0&&(s=1/Math.sqrt(s)),t[0]=e[0]*s,t[1]=e[1]*s,t[2]=e[2]*s,t}function dr(t,e){return t[0]*e[0]+t[1]*e[1]+t[2]*e[2]}function nt(t,e,n){let r=e[0],o=e[1],s=e[2],i=n[0],a=n[1],c=n[2];return t[0]=o*c-s*a,t[1]=s*i-r*c,t[2]=r*a-o*i,t}function rt(t,e,n){let r=e[0],o=e[1],s=e[2],i=n[3]*r+n[7]*o+n[11]*s+n[15];return i=i||1,t[0]=(n[0]*r+n[4]*o+n[8]*s+n[12])/i,t[1]=(n[1]*r+n[5]*o+n[9]*s+n[13])/i,t[2]=(n[2]*r+n[6]*o+n[10]*s+n[14])/i,t}function gr(t,e,n){let r=n[0],o=n[1],s=n[2],i=n[3],a=e[0],c=e[1],f=e[2],l=o*f-s*c,h=s*a-r*f,p=r*c-o*a,m=o*p-s*h,A=s*l-r*p,u=r*h-o*l,d=i*2;return l*=d,h*=d,p*=d,m*=2,A*=2,u*=2,t[0]=a+l+m,t[1]=c+h+A,t[2]=f+p+u,t}var Br=wi;var Bh=function(){let t=Kt();return function(e,n,r,o,s,i){let a,c;for(n||(n=3),r||(r=0),o?c=Math.min(o*n+r,e.length):c=e.length,a=r;a<c;a+=n)t[0]=e[a],t[1]=e[a+1],t[2]=e[a+2],s(t,t,i),e[a]=t[0],e[a+1]=t[1],e[a+2]=t[2];return e}}();var ot,ue=class extends tt{static get ZERO(){return ot||(ot=new ue(0,0,0,0),Object.freeze(ot)),ot}constructor(e=0,n=0,r=0,o=0){super(-0,-0,-0,-0),me(e)&&arguments.length===1?this.copy(e):(R.debug&&(y(e),y(n),y(r),y(o)),this[0]=e,this[1]=n,this[2]=r,this[3]=o)}set(e,n,r,o){return this[0]=e,this[1]=n,this[2]=r,this[3]=o,this.check()}copy(e){return this[0]=e[0],this[1]=e[1],this[2]=e[2],this[3]=e[3],this.check()}fromObject(e){return R.debug&&(y(e.x),y(e.y),y(e.z),y(e.w)),this[0]=e.x,this[1]=e.y,this[2]=e.z,this[3]=e.w,this}toObject(e){return e.x=this[0],e.y=this[1],e.z=this[2],e.w=this[3],e}get ELEMENTS(){return 4}get z(){return this[2]}set z(e){this[2]=y(e)}get w(){return this[3]}set w(e){this[3]=y(e)}transform(e){return rt(this,this,e),this.check()}transformByMatrix3(e){return Ar(this,this,e),this.check()}transformByMatrix2(e){return mr(this,this,e),this.check()}transformByQuaternion(e){return gr(this,this,e),this.check()}applyMatrix4(e){return e.transform(this,this),this}};var st=class extends ee{toString(){let e="[";if(R.printRowMajor){e+="row-major:";for(let n=0;n<this.RANK;++n)for(let r=0;r<this.RANK;++r)e+=` ${this[r*this.RANK+n]}`}else{e+="column-major:";for(let n=0;n<this.ELEMENTS;++n)e+=` ${this[n]}`}return e+="]",e}getElementIndex(e,n){return n*this.RANK+e}getElement(e,n){return this[n*this.RANK+e]}setElement(e,n,r){return this[n*this.RANK+e]=y(r),this}getColumn(e,n=new Array(this.RANK).fill(-0)){let r=e*this.RANK;for(let o=0;o<this.RANK;++o)n[o]=this[r+o];return n}setColumn(e,n){let r=e*this.RANK;for(let o=0;o<this.RANK;++o)this[r+o]=n[o];return this}};function xr(){let t=new S(9);return S!=Float32Array&&(t[1]=0,t[2]=0,t[3]=0,t[5]=0,t[6]=0,t[7]=0),t[0]=1,t[4]=1,t[8]=1,t}function Hi(t){return t[0]=1,t[1]=0,t[2]=0,t[3]=0,t[4]=0,t[5]=1,t[6]=0,t[7]=0,t[8]=0,t[9]=0,t[10]=1,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,t}function Mr(t,e){if(t===e){let n=e[1],r=e[2],o=e[3],s=e[6],i=e[7],a=e[11];t[1]=e[4],t[2]=e[8],t[3]=e[12],t[4]=n,t[6]=e[9],t[7]=e[13],t[8]=r,t[9]=s,t[11]=e[14],t[12]=o,t[13]=i,t[14]=a}else t[0]=e[0],t[1]=e[4],t[2]=e[8],t[3]=e[12],t[4]=e[1],t[5]=e[5],t[6]=e[9],t[7]=e[13],t[8]=e[2],t[9]=e[6],t[10]=e[10],t[11]=e[14],t[12]=e[3],t[13]=e[7],t[14]=e[11],t[15]=e[15];return t}function Cr(t,e){let n=e[0],r=e[1],o=e[2],s=e[3],i=e[4],a=e[5],c=e[6],f=e[7],l=e[8],h=e[9],p=e[10],m=e[11],A=e[12],u=e[13],d=e[14],B=e[15],E=n*a-r*i,g=n*c-o*i,x=n*f-s*i,M=r*c-o*a,C=r*f-s*a,v=o*f-s*c,G=l*u-h*A,O=l*d-p*A,P=l*B-m*A,J=h*d-p*u,K=h*B-m*u,V=p*B-m*d,_=E*V-g*K+x*J+M*P-C*O+v*G;return _?(_=1/_,t[0]=(a*V-c*K+f*J)*_,t[1]=(o*K-r*V-s*J)*_,t[2]=(u*v-d*C+B*M)*_,t[3]=(p*C-h*v-m*M)*_,t[4]=(c*P-i*V-f*O)*_,t[5]=(n*V-o*P+s*O)*_,t[6]=(d*x-A*v-B*g)*_,t[7]=(l*v-p*x+m*g)*_,t[8]=(i*K-a*P+f*G)*_,t[9]=(r*P-n*K-s*G)*_,t[10]=(A*C-u*x+B*E)*_,t[11]=(h*x-l*C-m*E)*_,t[12]=(a*O-i*J-c*G)*_,t[13]=(n*J-r*O+o*G)*_,t[14]=(u*g-A*M-d*E)*_,t[15]=(l*M-h*g+p*E)*_,t):null}function Er(t){let e=t[0],n=t[1],r=t[2],o=t[3],s=t[4],i=t[5],a=t[6],c=t[7],f=t[8],l=t[9],h=t[10],p=t[11],m=t[12],A=t[13],u=t[14],d=t[15],B=e*i-n*s,E=e*a-r*s,g=n*a-r*i,x=f*A-l*m,M=f*u-h*m,C=l*u-h*A,v=e*C-n*M+r*x,G=s*C-i*M+a*x,O=f*g-l*E+h*B,P=m*g-A*E+u*B;return c*v-o*G+d*O-p*P}function Xt(t,e,n){let r=e[0],o=e[1],s=e[2],i=e[3],a=e[4],c=e[5],f=e[6],l=e[7],h=e[8],p=e[9],m=e[10],A=e[11],u=e[12],d=e[13],B=e[14],E=e[15],g=n[0],x=n[1],M=n[2],C=n[3];return t[0]=g*r+x*a+M*h+C*u,t[1]=g*o+x*c+M*p+C*d,t[2]=g*s+x*f+M*m+C*B,t[3]=g*i+x*l+M*A+C*E,g=n[4],x=n[5],M=n[6],C=n[7],t[4]=g*r+x*a+M*h+C*u,t[5]=g*o+x*c+M*p+C*d,t[6]=g*s+x*f+M*m+C*B,t[7]=g*i+x*l+M*A+C*E,g=n[8],x=n[9],M=n[10],C=n[11],t[8]=g*r+x*a+M*h+C*u,t[9]=g*o+x*c+M*p+C*d,t[10]=g*s+x*f+M*m+C*B,t[11]=g*i+x*l+M*A+C*E,g=n[12],x=n[13],M=n[14],C=n[15],t[12]=g*r+x*a+M*h+C*u,t[13]=g*o+x*c+M*p+C*d,t[14]=g*s+x*f+M*m+C*B,t[15]=g*i+x*l+M*A+C*E,t}function br(t,e,n){let r=n[0],o=n[1],s=n[2],i,a,c,f,l,h,p,m,A,u,d,B;return e===t?(t[12]=e[0]*r+e[4]*o+e[8]*s+e[12],t[13]=e[1]*r+e[5]*o+e[9]*s+e[13],t[14]=e[2]*r+e[6]*o+e[10]*s+e[14],t[15]=e[3]*r+e[7]*o+e[11]*s+e[15]):(i=e[0],a=e[1],c=e[2],f=e[3],l=e[4],h=e[5],p=e[6],m=e[7],A=e[8],u=e[9],d=e[10],B=e[11],t[0]=i,t[1]=a,t[2]=c,t[3]=f,t[4]=l,t[5]=h,t[6]=p,t[7]=m,t[8]=A,t[9]=u,t[10]=d,t[11]=B,t[12]=i*r+l*o+A*s+e[12],t[13]=a*r+h*o+u*s+e[13],t[14]=c*r+p*o+d*s+e[14],t[15]=f*r+m*o+B*s+e[15]),t}function _r(t,e,n){let r=n[0],o=n[1],s=n[2];return t[0]=e[0]*r,t[1]=e[1]*r,t[2]=e[2]*r,t[3]=e[3]*r,t[4]=e[4]*o,t[5]=e[5]*o,t[6]=e[6]*o,t[7]=e[7]*o,t[8]=e[8]*s,t[9]=e[9]*s,t[10]=e[10]*s,t[11]=e[11]*s,t[12]=e[12],t[13]=e[13],t[14]=e[14],t[15]=e[15],t}function yr(t,e,n,r){let o=r[0],s=r[1],i=r[2],a=Math.sqrt(o*o+s*s+i*i),c,f,l,h,p,m,A,u,d,B,E,g,x,M,C,v,G,O,P,J,K,V,_,Le;return a<1e-6?null:(a=1/a,o*=a,s*=a,i*=a,f=Math.sin(n),c=Math.cos(n),l=1-c,h=e[0],p=e[1],m=e[2],A=e[3],u=e[4],d=e[5],B=e[6],E=e[7],g=e[8],x=e[9],M=e[10],C=e[11],v=o*o*l+c,G=s*o*l+i*f,O=i*o*l-s*f,P=o*s*l-i*f,J=s*s*l+c,K=i*s*l+o*f,V=o*i*l+s*f,_=s*i*l-o*f,Le=i*i*l+c,t[0]=h*v+u*G+g*O,t[1]=p*v+d*G+x*O,t[2]=m*v+B*G+M*O,t[3]=A*v+E*G+C*O,t[4]=h*P+u*J+g*K,t[5]=p*P+d*J+x*K,t[6]=m*P+B*J+M*K,t[7]=A*P+E*J+C*K,t[8]=h*V+u*_+g*Le,t[9]=p*V+d*_+x*Le,t[10]=m*V+B*_+M*Le,t[11]=A*V+E*_+C*Le,e!==t&&(t[12]=e[12],t[13]=e[13],t[14]=e[14],t[15]=e[15]),t)}function Tr(t,e,n){let r=Math.sin(n),o=Math.cos(n),s=e[4],i=e[5],a=e[6],c=e[7],f=e[8],l=e[9],h=e[10],p=e[11];return e!==t&&(t[0]=e[0],t[1]=e[1],t[2]=e[2],t[3]=e[3],t[12]=e[12],t[13]=e[13],t[14]=e[14],t[15]=e[15]),t[4]=s*o+f*r,t[5]=i*o+l*r,t[6]=a*o+h*r,t[7]=c*o+p*r,t[8]=f*o-s*r,t[9]=l*o-i*r,t[10]=h*o-a*r,t[11]=p*o-c*r,t}function Rr(t,e,n){let r=Math.sin(n),o=Math.cos(n),s=e[0],i=e[1],a=e[2],c=e[3],f=e[8],l=e[9],h=e[10],p=e[11];return e!==t&&(t[4]=e[4],t[5]=e[5],t[6]=e[6],t[7]=e[7],t[12]=e[12],t[13]=e[13],t[14]=e[14],t[15]=e[15]),t[0]=s*o-f*r,t[1]=i*o-l*r,t[2]=a*o-h*r,t[3]=c*o-p*r,t[8]=s*r+f*o,t[9]=i*r+l*o,t[10]=a*r+h*o,t[11]=c*r+p*o,t}function Ir(t,e,n){let r=Math.sin(n),o=Math.cos(n),s=e[0],i=e[1],a=e[2],c=e[3],f=e[4],l=e[5],h=e[6],p=e[7];return e!==t&&(t[8]=e[8],t[9]=e[9],t[10]=e[10],t[11]=e[11],t[12]=e[12],t[13]=e[13],t[14]=e[14],t[15]=e[15]),t[0]=s*o+f*r,t[1]=i*o+l*r,t[2]=a*o+h*r,t[3]=c*o+p*r,t[4]=f*o-s*r,t[5]=l*o-i*r,t[6]=h*o-a*r,t[7]=p*o-c*r,t}function Sr(t,e){let n=e[0],r=e[1],o=e[2],s=e[3],i=n+n,a=r+r,c=o+o,f=n*i,l=r*i,h=r*a,p=o*i,m=o*a,A=o*c,u=s*i,d=s*a,B=s*c;return t[0]=1-h-A,t[1]=l+B,t[2]=p-d,t[3]=0,t[4]=l-B,t[5]=1-f-A,t[6]=m+u,t[7]=0,t[8]=p+d,t[9]=m-u,t[10]=1-f-h,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,t}function Lr(t,e,n,r,o,s,i){let a=1/(n-e),c=1/(o-r),f=1/(s-i);return t[0]=s*2*a,t[1]=0,t[2]=0,t[3]=0,t[4]=0,t[5]=s*2*c,t[6]=0,t[7]=0,t[8]=(n+e)*a,t[9]=(o+r)*c,t[10]=(i+s)*f,t[11]=-1,t[12]=0,t[13]=0,t[14]=i*s*2*f,t[15]=0,t}function Ji(t,e,n,r,o){let s=1/Math.tan(e/2);if(t[0]=s/n,t[1]=0,t[2]=0,t[3]=0,t[4]=0,t[5]=s,t[6]=0,t[7]=0,t[8]=0,t[9]=0,t[11]=-1,t[12]=0,t[13]=0,t[15]=0,o!=null&&o!==1/0){let i=1/(r-o);t[10]=(o+r)*i,t[14]=2*o*r*i}else t[10]=-1,t[14]=-2*r;return t}var Fr=Ji;function Ki(t,e,n,r,o,s,i){let a=1/(e-n),c=1/(r-o),f=1/(s-i);return t[0]=-2*a,t[1]=0,t[2]=0,t[3]=0,t[4]=0,t[5]=-2*c,t[6]=0,t[7]=0,t[8]=0,t[9]=0,t[10]=2*f,t[11]=0,t[12]=(e+n)*a,t[13]=(o+r)*c,t[14]=(i+s)*f,t[15]=1,t}var Dr=Ki;function vr(t,e,n,r){let o,s,i,a,c,f,l,h,p,m,A=e[0],u=e[1],d=e[2],B=r[0],E=r[1],g=r[2],x=n[0],M=n[1],C=n[2];return Math.abs(A-x)<1e-6&&Math.abs(u-M)<1e-6&&Math.abs(d-C)<1e-6?Hi(t):(h=A-x,p=u-M,m=d-C,o=1/Math.sqrt(h*h+p*p+m*m),h*=o,p*=o,m*=o,s=E*m-g*p,i=g*h-B*m,a=B*p-E*h,o=Math.sqrt(s*s+i*i+a*a),o?(o=1/o,s*=o,i*=o,a*=o):(s=0,i=0,a=0),c=p*a-m*i,f=m*s-h*a,l=h*i-p*s,o=Math.sqrt(c*c+f*f+l*l),o?(o=1/o,c*=o,f*=o,l*=o):(c=0,f=0,l=0),t[0]=s,t[1]=c,t[2]=h,t[3]=0,t[4]=i,t[5]=f,t[6]=p,t[7]=0,t[8]=a,t[9]=l,t[10]=m,t[11]=0,t[12]=-(s*A+i*u+a*d),t[13]=-(c*A+f*u+l*d),t[14]=-(h*A+p*u+m*d),t[15]=1,t)}function Vi(){let t=new S(4);return S!=Float32Array&&(t[0]=0,t[1]=0,t[2]=0,t[3]=0),t}function Gr(t,e,n){return t[0]=e[0]+n[0],t[1]=e[1]+n[1],t[2]=e[2]+n[2],t[3]=e[3]+n[3],t}function Or(t,e,n){return t[0]=e[0]*n,t[1]=e[1]*n,t[2]=e[2]*n,t[3]=e[3]*n,t}function Pr(t){let e=t[0],n=t[1],r=t[2],o=t[3];return Math.sqrt(e*e+n*n+r*r+o*o)}function wr(t){let e=t[0],n=t[1],r=t[2],o=t[3];return e*e+n*n+r*r+o*o}function Nr(t,e){let n=e[0],r=e[1],o=e[2],s=e[3],i=n*n+r*r+o*o+s*s;return i>0&&(i=1/Math.sqrt(i)),t[0]=n*i,t[1]=r*i,t[2]=o*i,t[3]=s*i,t}function Ur(t,e){return t[0]*e[0]+t[1]*e[1]+t[2]*e[2]+t[3]*e[3]}function Hr(t,e,n,r){let o=e[0],s=e[1],i=e[2],a=e[3];return t[0]=o+r*(n[0]-o),t[1]=s+r*(n[1]-s),t[2]=i+r*(n[2]-i),t[3]=a+r*(n[3]-a),t}function Jr(t,e,n){let r=e[0],o=e[1],s=e[2],i=e[3];return t[0]=n[0]*r+n[4]*o+n[8]*s+n[12]*i,t[1]=n[1]*r+n[5]*o+n[9]*s+n[13]*i,t[2]=n[2]*r+n[6]*o+n[10]*s+n[14]*i,t[3]=n[3]*r+n[7]*o+n[11]*s+n[15]*i,t}function Kr(t,e,n){let r=e[0],o=e[1],s=e[2],i=n[0],a=n[1],c=n[2],f=n[3],l=f*r+a*s-c*o,h=f*o+c*r-i*s,p=f*s+i*o-a*r,m=-i*r-a*o-c*s;return t[0]=l*f+m*-i+h*-c-p*-a,t[1]=h*f+m*-a+p*-i-l*-c,t[2]=p*f+m*-c+l*-a-h*-i,t[3]=e[3],t}var Lh=function(){let t=Vi();return function(e,n,r,o,s,i){let a,c;for(n||(n=4),r||(r=0),o?c=Math.min(o*n+r,e.length):c=e.length,a=r;a<c;a+=n)t[0]=e[a],t[1]=e[a+1],t[2]=e[a+2],t[3]=e[a+3],s(t,t,i),e[a]=t[0],e[a+1]=t[1],e[a+2]=t[2],e[a+3]=t[3];return e}}();var zt;(function(t){t[t.COL0ROW0=0]="COL0ROW0",t[t.COL0ROW1=1]="COL0ROW1",t[t.COL0ROW2=2]="COL0ROW2",t[t.COL0ROW3=3]="COL0ROW3",t[t.COL1ROW0=4]="COL1ROW0",t[t.COL1ROW1=5]="COL1ROW1",t[t.COL1ROW2=6]="COL1ROW2",t[t.COL1ROW3=7]="COL1ROW3",t[t.COL2ROW0=8]="COL2ROW0",t[t.COL2ROW1=9]="COL2ROW1",t[t.COL2ROW2=10]="COL2ROW2",t[t.COL2ROW3=11]="COL2ROW3",t[t.COL3ROW0=12]="COL3ROW0",t[t.COL3ROW1=13]="COL3ROW1",t[t.COL3ROW2=14]="COL3ROW2",t[t.COL3ROW3=15]="COL3ROW3"})(zt||(zt={}));var ji=45*Math.PI/180,ki=1,jt=.1,kt=500,zi=Object.freeze([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]),q=class extends st{static get IDENTITY(){return Qi()}static get ZERO(){return Yi()}get ELEMENTS(){return 16}get RANK(){return 4}get INDICES(){return zt}constructor(e){super(-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0),arguments.length===1&&Array.isArray(e)?this.copy(e):this.identity()}copy(e){return this[0]=e[0],this[1]=e[1],this[2]=e[2],this[3]=e[3],this[4]=e[4],this[5]=e[5],this[6]=e[6],this[7]=e[7],this[8]=e[8],this[9]=e[9],this[10]=e[10],this[11]=e[11],this[12]=e[12],this[13]=e[13],this[14]=e[14],this[15]=e[15],this.check()}set(e,n,r,o,s,i,a,c,f,l,h,p,m,A,u,d){return this[0]=e,this[1]=n,this[2]=r,this[3]=o,this[4]=s,this[5]=i,this[6]=a,this[7]=c,this[8]=f,this[9]=l,this[10]=h,this[11]=p,this[12]=m,this[13]=A,this[14]=u,this[15]=d,this.check()}setRowMajor(e,n,r,o,s,i,a,c,f,l,h,p,m,A,u,d){return this[0]=e,this[1]=s,this[2]=f,this[3]=m,this[4]=n,this[5]=i,this[6]=l,this[7]=A,this[8]=r,this[9]=a,this[10]=h,this[11]=u,this[12]=o,this[13]=c,this[14]=p,this[15]=d,this.check()}toRowMajor(e){return e[0]=this[0],e[1]=this[4],e[2]=this[8],e[3]=this[12],e[4]=this[1],e[5]=this[5],e[6]=this[9],e[7]=this[13],e[8]=this[2],e[9]=this[6],e[10]=this[10],e[11]=this[14],e[12]=this[3],e[13]=this[7],e[14]=this[11],e[15]=this[15],e}identity(){return this.copy(zi)}fromObject(e){return this.check()}fromQuaternion(e){return Sr(this,e),this.check()}frustum(e){let{left:n,right:r,bottom:o,top:s,near:i=jt,far:a=kt}=e;return a===1/0?Wi(this,n,r,o,s,i):Lr(this,n,r,o,s,i,a),this.check()}lookAt(e){let{eye:n,center:r=[0,0,0],up:o=[0,1,0]}=e;return vr(this,n,r,o),this.check()}ortho(e){let{left:n,right:r,bottom:o,top:s,near:i=jt,far:a=kt}=e;return Dr(this,n,r,o,s,i,a),this.check()}orthographic(e){let{fovy:n=ji,aspect:r=ki,focalDistance:o=1,near:s=jt,far:i=kt}=e;Vr(n);let a=n/2,c=o*Math.tan(a),f=c*r;return this.ortho({left:-f,right:f,bottom:-c,top:c,near:s,far:i})}perspective(e){let{fovy:n=45*Math.PI/180,aspect:r=1,near:o=.1,far:s=500}=e;return Vr(n),Fr(this,n,r,o,s),this.check()}determinant(){return Er(this)}getScale(e=[-0,-0,-0]){return e[0]=Math.sqrt(this[0]*this[0]+this[1]*this[1]+this[2]*this[2]),e[1]=Math.sqrt(this[4]*this[4]+this[5]*this[5]+this[6]*this[6]),e[2]=Math.sqrt(this[8]*this[8]+this[9]*this[9]+this[10]*this[10]),e}getTranslation(e=[-0,-0,-0]){return e[0]=this[12],e[1]=this[13],e[2]=this[14],e}getRotation(e,n){e=e||[-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0],n=n||[-0,-0,-0];let r=this.getScale(n),o=1/r[0],s=1/r[1],i=1/r[2];return e[0]=this[0]*o,e[1]=this[1]*s,e[2]=this[2]*i,e[3]=0,e[4]=this[4]*o,e[5]=this[5]*s,e[6]=this[6]*i,e[7]=0,e[8]=this[8]*o,e[9]=this[9]*s,e[10]=this[10]*i,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,e}getRotationMatrix3(e,n){e=e||[-0,-0,-0,-0,-0,-0,-0,-0,-0],n=n||[-0,-0,-0];let r=this.getScale(n),o=1/r[0],s=1/r[1],i=1/r[2];return e[0]=this[0]*o,e[1]=this[1]*s,e[2]=this[2]*i,e[3]=this[4]*o,e[4]=this[5]*s,e[5]=this[6]*i,e[6]=this[8]*o,e[7]=this[9]*s,e[8]=this[10]*i,e}transpose(){return Mr(this,this),this.check()}invert(){return Cr(this,this),this.check()}multiplyLeft(e){return Xt(this,e,this),this.check()}multiplyRight(e){return Xt(this,this,e),this.check()}rotateX(e){return Tr(this,this,e),this.check()}rotateY(e){return Rr(this,this,e),this.check()}rotateZ(e){return Ir(this,this,e),this.check()}rotateXYZ(e){return this.rotateX(e[0]).rotateY(e[1]).rotateZ(e[2])}rotateAxis(e,n){return yr(this,this,e,n),this.check()}scale(e){return _r(this,this,Array.isArray(e)?e:[e,e,e]),this.check()}translate(e){return br(this,this,e),this.check()}transform(e,n){return e.length===4?(n=Jr(n||[-0,-0,-0,-0],e,this),Ae(n,4),n):this.transformAsPoint(e,n)}transformAsPoint(e,n){let{length:r}=e,o;switch(r){case 2:o=lr(n||[-0,-0],e,this);break;case 3:o=rt(n||[-0,-0,-0],e,this);break;default:throw new Error("Illegal vector")}return Ae(o,e.length),o}transformAsVector(e,n){let r;switch(e.length){case 2:r=hr(n||[-0,-0],e,this);break;case 3:r=pr(n||[-0,-0,-0],e,this);break;default:throw new Error("Illegal vector")}return Ae(r,e.length),r}transformPoint(e,n){return this.transformAsPoint(e,n)}transformVector(e,n){return this.transformAsPoint(e,n)}transformDirection(e,n){return this.transformAsVector(e,n)}makeRotationX(e){return this.identity().rotateX(e)}makeTranslation(e,n,r){return this.identity().translate([e,n,r])}},it,at;function Yi(){return it||(it=new q([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]),Object.freeze(it)),it}function Qi(){return at||(at=new q,Object.freeze(at)),at}function Vr(t){if(t>Math.PI*2)throw Error("expected radians")}function Wi(t,e,n,r,o,s){let i=2*s/(n-e),a=2*s/(o-r),c=(n+e)/(n-e),f=(o+r)/(o-r),l=-1,h=-1,p=-2*s;return t[0]=i,t[1]=0,t[2]=0,t[3]=0,t[4]=0,t[5]=a,t[6]=0,t[7]=0,t[8]=c,t[9]=f,t[10]=l,t[11]=h,t[12]=0,t[13]=0,t[14]=p,t[15]=0,t}function Xr(){let t=new S(4);return S!=Float32Array&&(t[0]=0,t[1]=0,t[2]=0),t[3]=1,t}function jr(t){return t[0]=0,t[1]=0,t[2]=0,t[3]=1,t}function Yt(t,e,n){n=n*.5;let r=Math.sin(n);return t[0]=r*e[0],t[1]=r*e[1],t[2]=r*e[2],t[3]=Math.cos(n),t}function Qt(t,e,n){let r=e[0],o=e[1],s=e[2],i=e[3],a=n[0],c=n[1],f=n[2],l=n[3];return t[0]=r*l+i*a+o*f-s*c,t[1]=o*l+i*c+s*a-r*f,t[2]=s*l+i*f+r*c-o*a,t[3]=i*l-r*a-o*c-s*f,t}function kr(t,e,n){n*=.5;let r=e[0],o=e[1],s=e[2],i=e[3],a=Math.sin(n),c=Math.cos(n);return t[0]=r*c+i*a,t[1]=o*c+s*a,t[2]=s*c-o*a,t[3]=i*c-r*a,t}function zr(t,e,n){n*=.5;let r=e[0],o=e[1],s=e[2],i=e[3],a=Math.sin(n),c=Math.cos(n);return t[0]=r*c-s*a,t[1]=o*c+i*a,t[2]=s*c+r*a,t[3]=i*c-o*a,t}function Yr(t,e,n){n*=.5;let r=e[0],o=e[1],s=e[2],i=e[3],a=Math.sin(n),c=Math.cos(n);return t[0]=r*c+o*a,t[1]=o*c-r*a,t[2]=s*c+i*a,t[3]=i*c-s*a,t}function Qr(t,e){let n=e[0],r=e[1],o=e[2];return t[0]=n,t[1]=r,t[2]=o,t[3]=Math.sqrt(Math.abs(1-n*n-r*r-o*o)),t}function ve(t,e,n,r){let o=e[0],s=e[1],i=e[2],a=e[3],c=n[0],f=n[1],l=n[2],h=n[3],p,m,A,u,d;return p=o*c+s*f+i*l+a*h,p<0&&(p=-p,c=-c,f=-f,l=-l,h=-h),1-p>1e-6?(m=Math.acos(p),d=Math.sin(m),A=Math.sin((1-r)*m)/d,u=Math.sin(r*m)/d):(A=1-r,u=r),t[0]=A*o+u*c,t[1]=A*s+u*f,t[2]=A*i+u*l,t[3]=A*a+u*h,t}function Wr(t,e){let n=e[0],r=e[1],o=e[2],s=e[3],i=n*n+r*r+o*o+s*s,a=i?1/i:0;return t[0]=-n*a,t[1]=-r*a,t[2]=-o*a,t[3]=s*a,t}function qr(t,e){return t[0]=-e[0],t[1]=-e[1],t[2]=-e[2],t[3]=e[3],t}function Wt(t,e){let n=e[0]+e[4]+e[8],r;if(n>0)r=Math.sqrt(n+1),t[3]=.5*r,r=.5/r,t[0]=(e[5]-e[7])*r,t[1]=(e[6]-e[2])*r,t[2]=(e[1]-e[3])*r;else{let o=0;e[4]>e[0]&&(o=1),e[8]>e[o*3+o]&&(o=2);let s=(o+1)%3,i=(o+2)%3;r=Math.sqrt(e[o*3+o]-e[s*3+s]-e[i*3+i]+1),t[o]=.5*r,r=.5/r,t[3]=(e[s*3+i]-e[i*3+s])*r,t[s]=(e[s*3+o]+e[o*3+s])*r,t[i]=(e[i*3+o]+e[o*3+i])*r}return t}var Zr=Gr;var $r=Or,eo=Ur,to=Hr,no=Pr;var ro=wr;var oo=Nr;var so=function(){let t=Kt(),e=Vt(1,0,0),n=Vt(0,1,0);return function(r,o,s){let i=dr(o,s);return i<-.999999?(nt(t,e,o),Br(t)<1e-6&&nt(t,n,o),ur(t,t),Yt(r,t,Math.PI),r):i>.999999?(r[0]=0,r[1]=0,r[2]=0,r[3]=1,r):(nt(t,o,s),r[0]=t[0],r[1]=t[1],r[2]=t[2],r[3]=1+i,oo(r,r))}}(),Xh=function(){let t=Xr(),e=Xr();return function(n,r,o,s,i,a){return ve(t,r,i,a),ve(e,o,s,a),ve(n,t,e,2*a*(1-a)),n}}(),jh=function(){let t=xr();return function(e,n,r,o){return t[0]=r[0],t[3]=r[1],t[6]=r[2],t[1]=o[0],t[4]=o[1],t[7]=o[2],t[2]=-n[0],t[5]=-n[1],t[8]=-n[2],oo(e,Wt(e,t))}}();var qi=[0,0,0,1],Ge=class extends ee{constructor(e=0,n=0,r=0,o=1){super(-0,-0,-0,-0),Array.isArray(e)&&arguments.length===1?this.copy(e):this.set(e,n,r,o)}copy(e){return this[0]=e[0],this[1]=e[1],this[2]=e[2],this[3]=e[3],this.check()}set(e,n,r,o){return this[0]=e,this[1]=n,this[2]=r,this[3]=o,this.check()}fromObject(e){return this[0]=e.x,this[1]=e.y,this[2]=e.z,this[3]=e.w,this.check()}fromMatrix3(e){return Wt(this,e),this.check()}fromAxisRotation(e,n){return Yt(this,e,n),this.check()}identity(){return jr(this),this.check()}setAxisAngle(e,n){return this.fromAxisRotation(e,n)}get ELEMENTS(){return 4}get x(){return this[0]}set x(e){this[0]=y(e)}get y(){return this[1]}set y(e){this[1]=y(e)}get z(){return this[2]}set z(e){this[2]=y(e)}get w(){return this[3]}set w(e){this[3]=y(e)}len(){return no(this)}lengthSquared(){return ro(this)}dot(e){return eo(this,e)}rotationTo(e,n){return so(this,e,n),this.check()}add(e){return Zr(this,this,e),this.check()}calculateW(){return Qr(this,this),this.check()}conjugate(){return qr(this,this),this.check()}invert(){return Wr(this,this),this.check()}lerp(e,n,r){return r===void 0?this.lerp(this,e,n):(to(this,e,n,r),this.check())}multiplyRight(e){return Qt(this,this,e),this.check()}multiplyLeft(e){return Qt(this,e,this),this.check()}normalize(){let e=this.len(),n=e>0?1/e:0;return this[0]=this[0]*n,this[1]=this[1]*n,this[2]=this[2]*n,this[3]=this[3]*n,e===0&&(this[3]=1),this.check()}rotateX(e){return kr(this,this,e),this.check()}rotateY(e){return zr(this,this,e),this.check()}rotateZ(e){return Yr(this,this,e),this.check()}scale(e){return $r(this,this,e),this.check()}slerp(e,n,r){let o,s,i;switch(arguments.length){case 1:({start:o=qi,target:s,ratio:i}=e);break;case 2:o=this,s=e,i=n;break;default:o=e,s=n,i=r}return ve(this,o,s,i),this.check()}transformVector4(e,n=new ue){return Kr(n,e,this),Ae(n,4)}lengthSq(){return this.lengthSquared()}setFromAxisAngle(e,n){return this.setAxisAngle(e,n)}premultiply(e){return this.multiplyLeft(e)}multiply(e){return this.multiplyRight(e)}};var ao=F(oe(),1);var qt=`precision highp int;

// #if (defined(SHADER_TYPE_FRAGMENT) && defined(LIGHTING_FRAGMENT)) || (defined(SHADER_TYPE_VERTEX) && defined(LIGHTING_VERTEX))
struct AmbientLight {
  vec3 color;
};

struct PointLight {
  vec3 color;
  vec3 position;
  vec3 attenuation; // 2nd order x:Constant-y:Linear-z:Exponential
};

struct DirectionalLight {
  vec3 color;
  vec3 direction;
};

uniform lightingUniforms {
  int enabled;
  int lightType;

  int directionalLightCount;
  int pointLightCount;

  vec3 ambientColor;

  vec3 lightColor0;
  vec3 lightPosition0;
  vec3 lightDirection0;
  vec3 lightAttenuation0;

  vec3 lightColor1;
  vec3 lightPosition1;
  vec3 lightDirection1;
  vec3 lightAttenuation1;

  vec3 lightColor2;
  vec3 lightPosition2;
  vec3 lightDirection2;
  vec3 lightAttenuation2;
} lighting;

PointLight lighting_getPointLight(int index) {
  switch (index) {
    case 0:
      return PointLight(lighting.lightColor0, lighting.lightPosition0, lighting.lightAttenuation0);
    case 1:
      return PointLight(lighting.lightColor1, lighting.lightPosition1, lighting.lightAttenuation1);
    case 2:
    default:  
      return PointLight(lighting.lightColor2, lighting.lightPosition2, lighting.lightAttenuation2);
  }
}

DirectionalLight lighting_getDirectionalLight(int index) {
  switch (index) {
    case 0:
      return DirectionalLight(lighting.lightColor0, lighting.lightDirection0);
    case 1:
      return DirectionalLight(lighting.lightColor1, lighting.lightDirection1);
    case 2:
    default:   
      return DirectionalLight(lighting.lightColor2, lighting.lightDirection2);
  }
} 

float getPointLightAttenuation(PointLight pointLight, float distance) {
  return pointLight.attenuation.x
       + pointLight.attenuation.y * distance
       + pointLight.attenuation.z * distance * distance;
}

// #endif
`;var io=`// #if (defined(SHADER_TYPE_FRAGMENT) && defined(LIGHTING_FRAGMENT)) || (defined(SHADER_TYPE_VERTEX) && defined(LIGHTING_VERTEX))
struct AmbientLight {
  color: vec3<f32>,
};

struct PointLight {
  color: vec3<f32>,
  position: vec3<f32>,
  attenuation: vec3<f32>, // 2nd order x:Constant-y:Linear-z:Exponential
};

struct DirectionalLight {
  color: vec3<f32>,
  direction: vec3<f32>,
};

struct lightingUniforms {
  enabled: i32,
  pointLightCount: i32,
  directionalLightCount: i32,

  ambientColor: vec3<f32>,

  // TODO - support multiple lights by uncommenting arrays below
  lightType: i32,
  lightColor: vec3<f32>,
  lightDirection: vec3<f32>,
  lightPosition: vec3<f32>,
  lightAttenuation: vec3<f32>,

  // AmbientLight ambientLight;
  // PointLight pointLight[MAX_LIGHTS];
  // DirectionalLight directionalLight[MAX_LIGHTS];
};

// Binding 0:1 is reserved for lighting (Note: could go into separate bind group as it is stable across draw calls)
@binding(1) @group(0) var<uniform> lighting : lightingUniforms;

fn lighting_getPointLight(index: i32) -> PointLight {
  return PointLight(lighting.lightColor, lighting.lightPosition, lighting.lightAttenuation);
}

fn lighting_getDirectionalLight(index: i32) -> DirectionalLight {
  return DirectionalLight(lighting.lightColor, lighting.lightDirection);
} 

fn getPointLightAttenuation(pointLight: PointLight, distance: f32) -> f32 {
  return pointLight.attenuation.x
       + pointLight.attenuation.y * distance
       + pointLight.attenuation.z * distance * distance;
}
`;var Zi=5,$i=255,Oe;(function(t){t[t.POINT=0]="POINT",t[t.DIRECTIONAL=1]="DIRECTIONAL"})(Oe||(Oe={}));var se={props:{},uniforms:{},name:"lighting",defines:{},uniformTypes:{enabled:"i32",lightType:"i32",directionalLightCount:"i32",pointLightCount:"i32",ambientColor:"vec3<f32>",lightColor0:"vec3<f32>",lightPosition0:"vec3<f32>",lightDirection0:"vec3<f32>",lightAttenuation0:"vec3<f32>",lightColor1:"vec3<f32>",lightPosition1:"vec3<f32>",lightDirection1:"vec3<f32>",lightAttenuation1:"vec3<f32>",lightColor2:"vec3<f32>",lightPosition2:"vec3<f32>",lightDirection2:"vec3<f32>",lightAttenuation2:"vec3<f32>"},defaultUniforms:{enabled:1,lightType:Oe.POINT,directionalLightCount:0,pointLightCount:0,ambientColor:[.1,.1,.1],lightColor0:[1,1,1],lightPosition0:[1,1,2],lightDirection0:[1,1,1],lightAttenuation0:[1,0,0],lightColor1:[1,1,1],lightPosition1:[1,1,2],lightDirection1:[1,1,1],lightAttenuation1:[1,0,0],lightColor2:[1,1,1],lightPosition2:[1,1,2],lightDirection2:[1,1,1],lightAttenuation2:[1,0,0]},source:io,vs:qt,fs:qt,getUniforms:ea};function ea(t,e={}){if(t=t&&{...t},!t)return{...se.defaultUniforms};t.lights&&(t={...t,...na(t.lights),lights:void 0});let{ambientLight:n,pointLights:r,directionalLights:o}=t||{};if(!(n||r&&r.length>0||o&&o.length>0))return{...se.defaultUniforms,enabled:0};let i={...se.defaultUniforms,...e,...ta({ambientLight:n,pointLights:r,directionalLights:o})};return t.enabled!==void 0&&(i.enabled=t.enabled?1:0),i}function ta({ambientLight:t,pointLights:e=[],directionalLights:n=[]}){let r={};r.ambientColor=Zt(t);let o=0;for(let s of e){r.lightType=Oe.POINT;let i=o;r[`lightColor${i}`]=Zt(s),r[`lightPosition${i}`]=s.position,r[`lightAttenuation${i}`]=s.attenuation||[1,0,0],o++}for(let s of n){r.lightType=Oe.DIRECTIONAL;let i=o;r[`lightColor${i}`]=Zt(s),r[`lightDirection${i}`]=s.direction,o++}return o>Zi&&ao.log.warn("MAX_LIGHTS exceeded")(),r.directionalLightCount=n.length,r.pointLightCount=e.length,r}function na(t){let e={pointLights:[],directionalLights:[]};for(let n of t||[])switch(n.type){case"ambient":e.ambientLight=n;break;case"directional":e.directionalLights?.push(n);break;case"point":e.pointLights?.push(n);break;default:}return e}function Zt(t={}){let{color:e=[0,0,0],intensity:n=1}=t;return e.map(r=>r*n/$i)}var co=`uniform phongMaterialUniforms {
  uniform float ambient;
  uniform float diffuse;
  uniform float shininess;
  uniform vec3  specularColor;
} material;
`,fo=`#define MAX_LIGHTS 3

uniform phongMaterialUniforms {
  uniform float ambient;
  uniform float diffuse;
  uniform float shininess;
  uniform vec3  specularColor;
} material;

vec3 lighting_getLightColor(vec3 surfaceColor, vec3 light_direction, vec3 view_direction, vec3 normal_worldspace, vec3 color) {
  vec3 halfway_direction = normalize(light_direction + view_direction);
  float lambertian = dot(light_direction, normal_worldspace);
  float specular = 0.0;
  if (lambertian > 0.0) {
    float specular_angle = max(dot(normal_worldspace, halfway_direction), 0.0);
    specular = pow(specular_angle, material.shininess);
  }
  lambertian = max(lambertian, 0.0);
  return (lambertian * material.diffuse * surfaceColor + specular * material.specularColor) * color;
}

vec3 lighting_getLightColor(vec3 surfaceColor, vec3 cameraPosition, vec3 position_worldspace, vec3 normal_worldspace) {
  vec3 lightColor = surfaceColor;

  if (lighting.enabled == 0) {
    return lightColor;
  }

  vec3 view_direction = normalize(cameraPosition - position_worldspace);
  lightColor = material.ambient * surfaceColor * lighting.ambientColor;

  for (int i = 0; i < lighting.pointLightCount; i++) {
    PointLight pointLight = lighting_getPointLight(i);
    vec3 light_position_worldspace = pointLight.position;
    vec3 light_direction = normalize(light_position_worldspace - position_worldspace);
    float light_attenuation = getPointLightAttenuation(pointLight, distance(light_position_worldspace, position_worldspace));
    lightColor += lighting_getLightColor(surfaceColor, light_direction, view_direction, normal_worldspace, pointLight.color / light_attenuation);
  }

  int totalLights = min(MAX_LIGHTS, lighting.pointLightCount + lighting.directionalLightCount);
  for (int i = lighting.pointLightCount; i < totalLights; i++) {
    DirectionalLight directionalLight = lighting_getDirectionalLight(i);
    lightColor += lighting_getLightColor(surfaceColor, -directionalLight.direction, view_direction, normal_worldspace, directionalLight.color);
  }
  
  return lightColor;
}
`;var lo=`struct phongMaterialUniforms {
  ambient: f32,
  diffuse: f32,
  shininess: f32,
  specularColor: vec3<f32>,
};

@binding(2) @group(0) var<uniform> phongMaterial : phongMaterialUniforms;

fn lighting_getLightColor(surfaceColor: vec3<f32>, light_direction: vec3<f32>, view_direction: vec3<f32>, normal_worldspace: vec3<f32>, color: vec3<f32>) -> vec3<f32> {
  let halfway_direction: vec3<f32> = normalize(light_direction + view_direction);
  var lambertian: f32 = dot(light_direction, normal_worldspace);
  var specular: f32 = 0.0;
  if (lambertian > 0.0) {
    let specular_angle = max(dot(normal_worldspace, halfway_direction), 0.0);
    specular = pow(specular_angle, phongMaterial.shininess);
  }
  lambertian = max(lambertian, 0.0);
  return (lambertian * phongMaterial.diffuse * surfaceColor + specular * phongMaterial.specularColor) * color;
}

fn lighting_getLightColor2(surfaceColor: vec3<f32>, cameraPosition: vec3<f32>, position_worldspace: vec3<f32>, normal_worldspace: vec3<f32>) -> vec3<f32> {
  var lightColor: vec3<f32> = surfaceColor;

  if (lighting.enabled == 0) {
    return lightColor;
  }

  let view_direction: vec3<f32> = normalize(cameraPosition - position_worldspace);
  lightColor = phongMaterial.ambient * surfaceColor * lighting.ambientColor;

  if (lighting.lightType == 0) {
    let pointLight: PointLight  = lighting_getPointLight(0);
    let light_position_worldspace: vec3<f32> = pointLight.position;
    let light_direction: vec3<f32> = normalize(light_position_worldspace - position_worldspace);
    lightColor += lighting_getLightColor(surfaceColor, light_direction, view_direction, normal_worldspace, pointLight.color);
  } else if (lighting.lightType == 1) {
    var directionalLight: DirectionalLight = lighting_getDirectionalLight(0);
    lightColor += lighting_getLightColor(surfaceColor, -directionalLight.direction, view_direction, normal_worldspace, directionalLight.color);
  }
  
  return lightColor;
  /*
  for (int i = 0; i < MAX_LIGHTS; i++) {
    if (i >= lighting.pointLightCount) {
      break;
    }
    PointLight pointLight = lighting.pointLight[i];
    vec3 light_position_worldspace = pointLight.position;
    vec3 light_direction = normalize(light_position_worldspace - position_worldspace);
    lightColor += lighting_getLightColor(surfaceColor, light_direction, view_direction, normal_worldspace, pointLight.color);
  }

  for (int i = 0; i < MAX_LIGHTS; i++) {
    if (i >= lighting.directionalLightCount) {
      break;
    }
    DirectionalLight directionalLight = lighting.directionalLight[i];
    lightColor += lighting_getLightColor(surfaceColor, -directionalLight.direction, view_direction, normal_worldspace, directionalLight.color);
  }
  */
}

fn lighting_getSpecularLightColor(cameraPosition: vec3<f32>, position_worldspace: vec3<f32>, normal_worldspace: vec3<f32>) -> vec3<f32>{
  var lightColor = vec3<f32>(0, 0, 0);
  let surfaceColor = vec3<f32>(0, 0, 0);

  if (lighting.enabled == 0) {
    let view_direction = normalize(cameraPosition - position_worldspace);

    switch (lighting.lightType) {
      case 0, default: {
        let pointLight: PointLight = lighting_getPointLight(0);
        let light_position_worldspace: vec3<f32> = pointLight.position;
        let light_direction: vec3<f32> = normalize(light_position_worldspace - position_worldspace);
        lightColor += lighting_getLightColor(surfaceColor, light_direction, view_direction, normal_worldspace, pointLight.color);
      }
      case 1: {
        let directionalLight: DirectionalLight = lighting_getDirectionalLight(0);
        lightColor += lighting_getLightColor(surfaceColor, -directionalLight.direction, view_direction, normal_worldspace, directionalLight.color);
      }
    }
  }
  return lightColor;
}
`;var ct={name:"phongMaterial",dependencies:[se],source:lo,vs:co,fs:fo,defines:{LIGHTING_FRAGMENT:!0},uniformTypes:{ambient:"f32",diffuse:"f32",shininess:"f32",specularColor:"vec3<f32>"},defaultUniforms:{ambient:.35,diffuse:.6,shininess:32,specularColor:[.15,.15,.15]},getUniforms(t){let e={...t};return e.specularColor&&(e.specularColor=e.specularColor.map(n=>n/255)),{...ct.defaultUniforms,...e}}};var ho=`out vec3 pbr_vPosition;
out vec2 pbr_vUV;

#ifdef HAS_NORMALS
# ifdef HAS_TANGENTS
out mat3 pbr_vTBN;
# else
out vec3 pbr_vNormal;
# endif
#endif

void pbr_setPositionNormalTangentUV(vec4 position, vec4 normal, vec4 tangent, vec2 uv)
{
  vec4 pos = pbrProjection.modelMatrix * position;
  pbr_vPosition = vec3(pos.xyz) / pos.w;

#ifdef HAS_NORMALS
#ifdef HAS_TANGENTS
  vec3 normalW = normalize(vec3(pbrProjection.normalMatrix * vec4(normal.xyz, 0.0)));
  vec3 tangentW = normalize(vec3(pbrProjection.modelMatrix * vec4(tangent.xyz, 0.0)));
  vec3 bitangentW = cross(normalW, tangentW) * tangent.w;
  pbr_vTBN = mat3(tangentW, bitangentW, normalW);
#else // HAS_TANGENTS != 1
  pbr_vNormal = normalize(vec3(pbrProjection.modelMatrix * vec4(normal.xyz, 0.0)));
#endif
#endif

#ifdef HAS_UV
  pbr_vUV = uv;
#else
  pbr_vUV = vec2(0.,0.);
#endif
}
`,po=`precision highp float;

uniform pbrMaterialUniforms {
  // Material is unlit
  bool unlit;

  // Base color map
  bool baseColorMapEnabled;
  vec4 baseColorFactor;

  bool normalMapEnabled;  
  float normalScale; // #ifdef HAS_NORMALMAP

  bool emissiveMapEnabled;
  vec3 emissiveFactor; // #ifdef HAS_EMISSIVEMAP

  vec2 metallicRoughnessValues;
  bool metallicRoughnessMapEnabled;

  bool occlusionMapEnabled;
  float occlusionStrength; // #ifdef HAS_OCCLUSIONMAP
  
  bool alphaCutoffEnabled;
  float alphaCutoff; // #ifdef ALPHA_CUTOFF
  
  // IBL
  bool IBLenabled;
  vec2 scaleIBLAmbient; // #ifdef USE_IBL
  
  // debugging flags used for shader output of intermediate PBR variables
  // #ifdef PBR_DEBUG
  vec4 scaleDiffBaseMR;
  vec4 scaleFGDSpec;
  // #endif
} pbrMaterial;

// Samplers
#ifdef HAS_BASECOLORMAP
uniform sampler2D pbr_baseColorSampler;
#endif
#ifdef HAS_NORMALMAP
uniform sampler2D pbr_normalSampler;
#endif
#ifdef HAS_EMISSIVEMAP
uniform sampler2D pbr_emissiveSampler;
#endif
#ifdef HAS_METALROUGHNESSMAP
uniform sampler2D pbr_metallicRoughnessSampler;
#endif
#ifdef HAS_OCCLUSIONMAP
uniform sampler2D pbr_occlusionSampler;
#endif
#ifdef USE_IBL
uniform samplerCube pbr_diffuseEnvSampler;
uniform samplerCube pbr_specularEnvSampler;
uniform sampler2D pbr_brdfLUT;
#endif

// Inputs from vertex shader

in vec3 pbr_vPosition;
in vec2 pbr_vUV;

#ifdef HAS_NORMALS
#ifdef HAS_TANGENTS
in mat3 pbr_vTBN;
#else
in vec3 pbr_vNormal;
#endif
#endif

// Encapsulate the various inputs used by the various functions in the shading equation
// We store values in this struct to simplify the integration of alternative implementations
// of the shading terms, outlined in the Readme.MD Appendix.
struct PBRInfo {
  float NdotL;                  // cos angle between normal and light direction
  float NdotV;                  // cos angle between normal and view direction
  float NdotH;                  // cos angle between normal and half vector
  float LdotH;                  // cos angle between light direction and half vector
  float VdotH;                  // cos angle between view direction and half vector
  float perceptualRoughness;    // roughness value, as authored by the model creator (input to shader)
  float metalness;              // metallic value at the surface
  vec3 reflectance0;            // full reflectance color (normal incidence angle)
  vec3 reflectance90;           // reflectance color at grazing angle
  float alphaRoughness;         // roughness mapped to a more linear change in the roughness (proposed by [2])
  vec3 diffuseColor;            // color contribution from diffuse lighting
  vec3 specularColor;           // color contribution from specular lighting
  vec3 n;                       // normal at surface point
  vec3 v;                       // vector from surface point to camera
};

const float M_PI = 3.141592653589793;
const float c_MinRoughness = 0.04;

vec4 SRGBtoLINEAR(vec4 srgbIn)
{
#ifdef MANUAL_SRGB
#ifdef SRGB_FAST_APPROXIMATION
  vec3 linOut = pow(srgbIn.xyz,vec3(2.2));
#else // SRGB_FAST_APPROXIMATION
  vec3 bLess = step(vec3(0.04045),srgbIn.xyz);
  vec3 linOut = mix( srgbIn.xyz/vec3(12.92), pow((srgbIn.xyz+vec3(0.055))/vec3(1.055),vec3(2.4)), bLess );
#endif //SRGB_FAST_APPROXIMATION
  return vec4(linOut,srgbIn.w);;
#else //MANUAL_SRGB
  return srgbIn;
#endif //MANUAL_SRGB
}

// Find the normal for this fragment, pulling either from a predefined normal map
// or from the interpolated mesh normal and tangent attributes.
vec3 getNormal()
{
  // Retrieve the tangent space matrix
#ifndef HAS_TANGENTS
  vec3 pos_dx = dFdx(pbr_vPosition);
  vec3 pos_dy = dFdy(pbr_vPosition);
  vec3 tex_dx = dFdx(vec3(pbr_vUV, 0.0));
  vec3 tex_dy = dFdy(vec3(pbr_vUV, 0.0));
  vec3 t = (tex_dy.t * pos_dx - tex_dx.t * pos_dy) / (tex_dx.s * tex_dy.t - tex_dy.s * tex_dx.t);

#ifdef HAS_NORMALS
  vec3 ng = normalize(pbr_vNormal);
#else
  vec3 ng = cross(pos_dx, pos_dy);
#endif

  t = normalize(t - ng * dot(ng, t));
  vec3 b = normalize(cross(ng, t));
  mat3 tbn = mat3(t, b, ng);
#else // HAS_TANGENTS
  mat3 tbn = pbr_vTBN;
#endif

#ifdef HAS_NORMALMAP
  vec3 n = texture(pbr_normalSampler, pbr_vUV).rgb;
  n = normalize(tbn * ((2.0 * n - 1.0) * vec3(pbrMaterial.normalScale, pbrMaterial.normalScale, 1.0)));
#else
  // The tbn matrix is linearly interpolated, so we need to re-normalize
  vec3 n = normalize(tbn[2].xyz);
#endif

  return n;
}

// Calculation of the lighting contribution from an optional Image Based Light source.
// Precomputed Environment Maps are required uniform inputs and are computed as outlined in [1].
// See our README.md on Environment Maps [3] for additional discussion.
#ifdef USE_IBL
vec3 getIBLContribution(PBRInfo pbrInfo, vec3 n, vec3 reflection)
{
  float mipCount = 9.0; // resolution of 512x512
  float lod = (pbrInfo.perceptualRoughness * mipCount);
  // retrieve a scale and bias to F0. See [1], Figure 3
  vec3 brdf = SRGBtoLINEAR(texture(pbr_brdfLUT,
    vec2(pbrInfo.NdotV, 1.0 - pbrInfo.perceptualRoughness))).rgb;
  vec3 diffuseLight = SRGBtoLINEAR(texture(pbr_diffuseEnvSampler, n)).rgb;

#ifdef USE_TEX_LOD
  vec3 specularLight = SRGBtoLINEAR(texture(pbr_specularEnvSampler, reflection, lod)).rgb;
#else
  vec3 specularLight = SRGBtoLINEAR(texture(pbr_specularEnvSampler, reflection)).rgb;
#endif

  vec3 diffuse = diffuseLight * pbrInfo.diffuseColor;
  vec3 specular = specularLight * (pbrInfo.specularColor * brdf.x + brdf.y);

  // For presentation, this allows us to disable IBL terms
  diffuse *= pbrMaterial.scaleIBLAmbient.x;
  specular *= pbrMaterial.scaleIBLAmbient.y;

  return diffuse + specular;
}
#endif

// Basic Lambertian diffuse
// Implementation from Lambert's Photometria https://archive.org/details/lambertsphotome00lambgoog
// See also [1], Equation 1
vec3 diffuse(PBRInfo pbrInfo)
{
  return pbrInfo.diffuseColor / M_PI;
}

// The following equation models the Fresnel reflectance term of the spec equation (aka F())
// Implementation of fresnel from [4], Equation 15
vec3 specularReflection(PBRInfo pbrInfo)
{
  return pbrInfo.reflectance0 +
    (pbrInfo.reflectance90 - pbrInfo.reflectance0) *
    pow(clamp(1.0 - pbrInfo.VdotH, 0.0, 1.0), 5.0);
}

// This calculates the specular geometric attenuation (aka G()),
// where rougher material will reflect less light back to the viewer.
// This implementation is based on [1] Equation 4, and we adopt their modifications to
// alphaRoughness as input as originally proposed in [2].
float geometricOcclusion(PBRInfo pbrInfo)
{
  float NdotL = pbrInfo.NdotL;
  float NdotV = pbrInfo.NdotV;
  float r = pbrInfo.alphaRoughness;

  float attenuationL = 2.0 * NdotL / (NdotL + sqrt(r * r + (1.0 - r * r) * (NdotL * NdotL)));
  float attenuationV = 2.0 * NdotV / (NdotV + sqrt(r * r + (1.0 - r * r) * (NdotV * NdotV)));
  return attenuationL * attenuationV;
}

// The following equation(s) model the distribution of microfacet normals across
// the area being drawn (aka D())
// Implementation from "Average Irregularity Representation of a Roughened Surface
// for Ray Reflection" by T. S. Trowbridge, and K. P. Reitz
// Follows the distribution function recommended in the SIGGRAPH 2013 course notes
// from EPIC Games [1], Equation 3.
float microfacetDistribution(PBRInfo pbrInfo)
{
  float roughnessSq = pbrInfo.alphaRoughness * pbrInfo.alphaRoughness;
  float f = (pbrInfo.NdotH * roughnessSq - pbrInfo.NdotH) * pbrInfo.NdotH + 1.0;
  return roughnessSq / (M_PI * f * f);
}

void PBRInfo_setAmbientLight(inout PBRInfo pbrInfo) {
  pbrInfo.NdotL = 1.0;
  pbrInfo.NdotH = 0.0;
  pbrInfo.LdotH = 0.0;
  pbrInfo.VdotH = 1.0;
}

void PBRInfo_setDirectionalLight(inout PBRInfo pbrInfo, vec3 lightDirection) {
  vec3 n = pbrInfo.n;
  vec3 v = pbrInfo.v;
  vec3 l = normalize(lightDirection);             // Vector from surface point to light
  vec3 h = normalize(l+v);                        // Half vector between both l and v

  pbrInfo.NdotL = clamp(dot(n, l), 0.001, 1.0);
  pbrInfo.NdotH = clamp(dot(n, h), 0.0, 1.0);
  pbrInfo.LdotH = clamp(dot(l, h), 0.0, 1.0);
  pbrInfo.VdotH = clamp(dot(v, h), 0.0, 1.0);
}

void PBRInfo_setPointLight(inout PBRInfo pbrInfo, PointLight pointLight) {
  vec3 light_direction = normalize(pointLight.position - pbr_vPosition);
  PBRInfo_setDirectionalLight(pbrInfo, light_direction);
}

vec3 calculateFinalColor(PBRInfo pbrInfo, vec3 lightColor) {
  // Calculate the shading terms for the microfacet specular shading model
  vec3 F = specularReflection(pbrInfo);
  float G = geometricOcclusion(pbrInfo);
  float D = microfacetDistribution(pbrInfo);

  // Calculation of analytical lighting contribution
  vec3 diffuseContrib = (1.0 - F) * diffuse(pbrInfo);
  vec3 specContrib = F * G * D / (4.0 * pbrInfo.NdotL * pbrInfo.NdotV);
  // Obtain final intensity as reflectance (BRDF) scaled by the energy of the light (cosine law)
  return pbrInfo.NdotL * lightColor * (diffuseContrib + specContrib);
}

vec4 pbr_filterColor(vec4 colorUnused)
{
  // The albedo may be defined from a base texture or a flat color
#ifdef HAS_BASECOLORMAP
  vec4 baseColor = SRGBtoLINEAR(texture(pbr_baseColorSampler, pbr_vUV)) * pbrMaterial.baseColorFactor;
#else
  vec4 baseColor = pbrMaterial.baseColorFactor;
#endif

#ifdef ALPHA_CUTOFF
  if (baseColor.a < pbrMaterial.alphaCutoff) {
    discard;
  }
#endif

  vec3 color = vec3(0, 0, 0);

  if(pbrMaterial.unlit){
    color.rgb = baseColor.rgb;
  }
  else{
    // Metallic and Roughness material properties are packed together
    // In glTF, these factors can be specified by fixed scalar values
    // or from a metallic-roughness map
    float perceptualRoughness = pbrMaterial.metallicRoughnessValues.y;
    float metallic = pbrMaterial.metallicRoughnessValues.x;
#ifdef HAS_METALROUGHNESSMAP
    // Roughness is stored in the 'g' channel, metallic is stored in the 'b' channel.
    // This layout intentionally reserves the 'r' channel for (optional) occlusion map data
    vec4 mrSample = texture(pbr_metallicRoughnessSampler, pbr_vUV);
    perceptualRoughness = mrSample.g * perceptualRoughness;
    metallic = mrSample.b * metallic;
#endif
    perceptualRoughness = clamp(perceptualRoughness, c_MinRoughness, 1.0);
    metallic = clamp(metallic, 0.0, 1.0);
    // Roughness is authored as perceptual roughness; as is convention,
    // convert to material roughness by squaring the perceptual roughness [2].
    float alphaRoughness = perceptualRoughness * perceptualRoughness;

    vec3 f0 = vec3(0.04);
    vec3 diffuseColor = baseColor.rgb * (vec3(1.0) - f0);
    diffuseColor *= 1.0 - metallic;
    vec3 specularColor = mix(f0, baseColor.rgb, metallic);

    // Compute reflectance.
    float reflectance = max(max(specularColor.r, specularColor.g), specularColor.b);

    // For typical incident reflectance range (between 4% to 100%) set the grazing
    // reflectance to 100% for typical fresnel effect.
    // For very low reflectance range on highly diffuse objects (below 4%),
    // incrementally reduce grazing reflecance to 0%.
    float reflectance90 = clamp(reflectance * 25.0, 0.0, 1.0);
    vec3 specularEnvironmentR0 = specularColor.rgb;
    vec3 specularEnvironmentR90 = vec3(1.0, 1.0, 1.0) * reflectance90;

    vec3 n = getNormal();                          // normal at surface point
    vec3 v = normalize(pbrProjection.camera - pbr_vPosition);  // Vector from surface point to camera

    float NdotV = clamp(abs(dot(n, v)), 0.001, 1.0);
    vec3 reflection = -normalize(reflect(v, n));

    PBRInfo pbrInfo = PBRInfo(
      0.0, // NdotL
      NdotV,
      0.0, // NdotH
      0.0, // LdotH
      0.0, // VdotH
      perceptualRoughness,
      metallic,
      specularEnvironmentR0,
      specularEnvironmentR90,
      alphaRoughness,
      diffuseColor,
      specularColor,
      n,
      v
    );


#ifdef USE_LIGHTS
    // Apply ambient light
    PBRInfo_setAmbientLight(pbrInfo);
    color += calculateFinalColor(pbrInfo, lighting.ambientColor);

    // Apply directional light
    for(int i = 0; i < lighting.directionalLightCount; i++) {
      if (i < lighting.directionalLightCount) {
        PBRInfo_setDirectionalLight(pbrInfo, lighting_getDirectionalLight(i).direction);
        color += calculateFinalColor(pbrInfo, lighting_getDirectionalLight(i).color);
      }
    }

    // Apply point light
    for(int i = 0; i < lighting.pointLightCount; i++) {
      if (i < lighting.pointLightCount) {
        PBRInfo_setPointLight(pbrInfo, lighting_getPointLight(i));
        float attenuation = getPointLightAttenuation(lighting_getPointLight(i), distance(lighting_getPointLight(i).position, pbr_vPosition));
        color += calculateFinalColor(pbrInfo, lighting_getPointLight(i).color / attenuation);
      }
    }
#endif

    // Calculate lighting contribution from image based lighting source (IBL)
#ifdef USE_IBL
    if (pbrMaterial.IBLenabled) {
      color += getIBLContribution(pbrInfo, n, reflection);
    }
#endif

 // Apply optional PBR terms for additional (optional) shading
#ifdef HAS_OCCLUSIONMAP
    if (pbrMaterial.occlusionMapEnabled) {
      float ao = texture(pbr_occlusionSampler, pbr_vUV).r;
      color = mix(color, color * ao, pbrMaterial.occlusionStrength);
    }
#endif

#ifdef HAS_EMISSIVEMAP
    if (pbrMaterial.emissiveMapEnabled) {
      vec3 emissive = SRGBtoLINEAR(texture(pbr_emissiveSampler, pbr_vUV)).rgb * pbrMaterial.emissiveFactor;
      color += emissive;
    }
#endif

    // This section uses mix to override final color for reference app visualization
    // of various parameters in the lighting equation.
#ifdef PBR_DEBUG
    // TODO: Figure out how to debug multiple lights

    // color = mix(color, F, pbr_scaleFGDSpec.x);
    // color = mix(color, vec3(G), pbr_scaleFGDSpec.y);
    // color = mix(color, vec3(D), pbr_scaleFGDSpec.z);
    // color = mix(color, specContrib, pbr_scaleFGDSpec.w);

    // color = mix(color, diffuseContrib, pbr_scaleDiffBaseMR.x);
    color = mix(color, baseColor.rgb, pbrMaterial.scaleDiffBaseMR.y);
    color = mix(color, vec3(metallic), pbrMaterial.scaleDiffBaseMR.z);
    color = mix(color, vec3(perceptualRoughness), pbrMaterial.scaleDiffBaseMR.w);
#endif

  }

  return vec4(pow(color,vec3(1.0/2.2)), baseColor.a);
}
`;var mo=`struct PBRFragmentInputs {
  pbr_vPosition: vec3f,
  pbr_vUV: vec2f,
  pbr_vTBN: mat3f,
  pbr_vNormal: vec3f
};

var fragmentInputs: PBRFragmentInputs;

fn pbr_setPositionNormalTangentUV(position: vec4f, normal: vec4f, tangent: vec4f, uv: vec2f)
{
  var pos: vec4f = pbrProjection.modelMatrix * position;
  pbr_vPosition = vec3(pos.xyz) / pos.w;

#ifdef HAS_NORMALS
#ifdef HAS_TANGENTS
  let normalW: vec3f = normalize(vec3(pbrProjection.normalMatrix * vec4(normal.xyz, 0.0)));
  let tangentW: vec3f = normalize(vec3(pbrProjection.modelMatrix * vec4(tangent.xyz, 0.0)));
  let bitangentW: vec3f = cross(normalW, tangentW) * tangent.w;
  fragmentInputs,pbr_vTBN = mat3(tangentW, bitangentW, normalW);
#else // HAS_TANGENTS != 1
  fragmentInputs.pbr_vNormal = normalize(vec3(pbrProjection.modelMatrix * vec4(normal.xyz, 0.0)));
#endif
#endif

#ifdef HAS_UV
  pbr_vUV = uv;
#else
  pbr_vUV = vec2(0.,0.);
#endif
}

struct pbrMaterialUniforms {
  // Material is unlit
  unlit: uint32,

  // Base color map
  baseColorMapEnabled: uint32,
  baseColorFactor: vec4f,

  normalMapEnabled : uint32,
  normalScale: f32,  // #ifdef HAS_NORMALMAP

  emissiveMapEnabled: uint32,
  emissiveFactor: vec3f, // #ifdef HAS_EMISSIVEMAP

  metallicRoughnessValues: vec2f,
  metallicRoughnessMapEnabled: uint32,

  occlusionMapEnabled: i32,
  occlusionStrength: f32, // #ifdef HAS_OCCLUSIONMAP
  
  alphaCutoffEnabled: i32,
  alphaCutoff: f32, // #ifdef ALPHA_CUTOFF
  
  // IBL
  IBLenabled: i32,
  scaleIBLAmbient: vec2f, // #ifdef USE_IBL
  
  // debugging flags used for shader output of intermediate PBR variables
  // #ifdef PBR_DEBUG
  scaleDiffBaseMR: vec4f,
  scaleFGDSpec: vec4f
  // #endif
} 
  
@binding(2) @group(0) var<uniform> material : pbrMaterialUniforms;

// Samplers
#ifdef HAS_BASECOLORMAP
uniform sampler2D pbr_baseColorSampler;
#endif
#ifdef HAS_NORMALMAP
uniform sampler2D pbr_normalSampler;
#endif
#ifdef HAS_EMISSIVEMAP
uniform sampler2D pbr_emissiveSampler;
#endif
#ifdef HAS_METALROUGHNESSMAP
uniform sampler2D pbr_metallicRoughnessSampler;
#endif
#ifdef HAS_OCCLUSIONMAP
uniform sampler2D pbr_occlusionSampler;
#endif
#ifdef USE_IBL
uniform samplerCube pbr_diffuseEnvSampler;
uniform samplerCube pbr_specularEnvSampler;
uniform sampler2D pbr_brdfLUT;
#endif

// Encapsulate the various inputs used by the various functions in the shading equation
// We store values in this struct to simplify the integration of alternative implementations
// of the shading terms, outlined in the Readme.MD Appendix.
struct PBRInfo {
  NdotL: f32,                  // cos angle between normal and light direction
  NdotV: f32,                  // cos angle between normal and view direction
  NdotH: f32,                  // cos angle between normal and half vector
  LdotH: f32,                  // cos angle between light direction and half vector
  VdotH: f32,                  // cos angle between view direction and half vector
  perceptualRoughness: f32,    // roughness value, as authored by the model creator (input to shader)
  metalness: f32,              // metallic value at the surface
  reflectance0: vec3f,            // full reflectance color (normal incidence angle)
  reflectance90: vec3f,           // reflectance color at grazing angle
  alphaRoughness: f32,         // roughness mapped to a more linear change in the roughness (proposed by [2])
  diffuseColor: vec3f,            // color contribution from diffuse lighting
  specularColor: vec3f,           // color contribution from specular lighting
  n: vec3f,                       // normal at surface point
  v: vec3f,                       // vector from surface point to camera
};

const M_PI = 3.141592653589793;
const c_MinRoughness = 0.04;

fn SRGBtoLINEAR(srgbIn: vec4f ) -> vec4f
{
#ifdef MANUAL_SRGB
#ifdef SRGB_FAST_APPROXIMATION
  var linOut: vec3f = pow(srgbIn.xyz,vec3(2.2));
#else // SRGB_FAST_APPROXIMATION
  var bLess: vec3f = step(vec3(0.04045),srgbIn.xyz);
  var linOut: vec3f = mix( srgbIn.xyz/vec3(12.92), pow((srgbIn.xyz+vec3(0.055))/vec3(1.055),vec3(2.4)), bLess );
#endif //SRGB_FAST_APPROXIMATION
  return vec4f(linOut,srgbIn.w);;
#else //MANUAL_SRGB
  return srgbIn;
#endif //MANUAL_SRGB
}

// Find the normal for this fragment, pulling either from a predefined normal map
// or from the interpolated mesh normal and tangent attributes.
fn getNormal() -> vec3f
{
  // Retrieve the tangent space matrix
#ifndef HAS_TANGENTS
  var pos_dx: vec3f = dFdx(pbr_vPosition);
  var pos_dy: vec3f = dFdy(pbr_vPosition);
  var tex_dx: vec3f = dFdx(vec3(pbr_vUV, 0.0));
  var tex_dy: vec3f = dFdy(vec3(pbr_vUV, 0.0));
  var t: vec3f = (tex_dy.t * pos_dx - tex_dx.t * pos_dy) / (tex_dx.s * tex_dy.t - tex_dy.s * tex_dx.t);

#ifdef HAS_NORMALS
  var ng: vec3f = normalize(pbr_vNormal);
#else
  var ng: vec3f = cross(pos_dx, pos_dy);
#endif

  t = normalize(t - ng * dot(ng, t));
  var b: vec3f = normalize(cross(ng, t));
  var tbn: mat3f = mat3f(t, b, ng);
#else // HAS_TANGENTS
  var tbn: mat3f = pbr_vTBN;
#endif

#ifdef HAS_NORMALMAP
  vec3 n = texture(pbr_normalSampler, pbr_vUV).rgb;
  n = normalize(tbn * ((2.0 * n - 1.0) * vec3(pbrMaterial.normalScale, pbrMaterial.normalScale, 1.0)));
#else
  // The tbn matrix is linearly interpolated, so we need to re-normalize
  vec3 n = normalize(tbn[2].xyz);
#endif

  return n;
}

// Calculation of the lighting contribution from an optional Image Based Light source.
// Precomputed Environment Maps are required uniform inputs and are computed as outlined in [1].
// See our README.md on Environment Maps [3] for additional discussion.
#ifdef USE_IBL
fn getIBLContribution(PBRInfo pbrInfo, vec3 n, vec3 reflection) -> vec3f
{
  float mipCount = 9.0; // resolution of 512x512
  float lod = (pbrInfo.perceptualRoughness * mipCount);
  // retrieve a scale and bias to F0. See [1], Figure 3
  vec3 brdf = SRGBtoLINEAR(texture(pbr_brdfLUT,
    vec2(pbrInfo.NdotV, 1.0 - pbrInfo.perceptualRoughness))).rgb;
  vec3 diffuseLight = SRGBtoLINEAR(texture(pbr_diffuseEnvSampler, n)).rgb;

#ifdef USE_TEX_LOD
  vec3 specularLight = SRGBtoLINEAR(texture(pbr_specularEnvSampler, reflection, lod)).rgb;
#else
  vec3 specularLight = SRGBtoLINEAR(texture(pbr_specularEnvSampler, reflection)).rgb;
#endif

  vec3 diffuse = diffuseLight * pbrInfo.diffuseColor;
  vec3 specular = specularLight * (pbrInfo.specularColor * brdf.x + brdf.y);

  // For presentation, this allows us to disable IBL terms
  diffuse *= pbrMaterial.scaleIBLAmbient.x;
  specular *= pbrMaterial.scaleIBLAmbient.y;

  return diffuse + specular;
}
#endif

// Basic Lambertian diffuse
// Implementation from Lambert's Photometria https://archive.org/details/lambertsphotome00lambgoog
// See also [1], Equation 1
fn diffuse(pbrInfo: PBRInfo) -> vec3<f32> {
  return pbrInfo.diffuseColor / PI;
}

// The following equation models the Fresnel reflectance term of the spec equation (aka F())
// Implementation of fresnel from [4], Equation 15
fn specularReflection(pbrInfo: PBRInfo) -> vec3<f32> {
  return pbrInfo.reflectance0 +
    (pbrInfo.reflectance90 - pbrInfo.reflectance0) *
    pow(clamp(1.0 - pbrInfo.VdotH, 0.0, 1.0), 5.0);
}

// This calculates the specular geometric attenuation (aka G()),
// where rougher material will reflect less light back to the viewer.
// This implementation is based on [1] Equation 4, and we adopt their modifications to
// alphaRoughness as input as originally proposed in [2].
fn geometricOcclusion(pbrInfo: PBRInfo) -> f32 {
  let NdotL: f32 = pbrInfo.NdotL;
  let NdotV: f32 = pbrInfo.NdotV;
  let r: f32 = pbrInfo.alphaRoughness;

  let attenuationL = 2.0 * NdotL / (NdotL + sqrt(r * r + (1.0 - r * r) * (NdotL * NdotL)));
  let attenuationV = 2.0 * NdotV / (NdotV + sqrt(r * r + (1.0 - r * r) * (NdotV * NdotV)));
  return attenuationL * attenuationV;
}

// The following equation(s) model the distribution of microfacet normals across
// the area being drawn (aka D())
// Implementation from "Average Irregularity Representation of a Roughened Surface
// for Ray Reflection" by T. S. Trowbridge, and K. P. Reitz
// Follows the distribution function recommended in the SIGGRAPH 2013 course notes
// from EPIC Games [1], Equation 3.
fn microfacetDistribution(pbrInfo: PBRInfo) -> f32 {
  let roughnessSq = pbrInfo.alphaRoughness * pbrInfo.alphaRoughness;
  let f = (pbrInfo.NdotH * roughnessSq - pbrInfo.NdotH) * pbrInfo.NdotH + 1.0;
  return roughnessSq / (PI * f * f);
}

fn PBRInfo_setAmbientLight(pbrInfo: ptr<function, PBRInfo>) {
  (*pbrInfo).NdotL = 1.0;
  (*pbrInfo).NdotH = 0.0;
  (*pbrInfo).LdotH = 0.0;
  (*pbrInfo).VdotH = 1.0;
}

fn PBRInfo_setDirectionalLight(pbrInfo: ptr<function, PBRInfo>, lightDirection: vec3<f32>) {
  let n = (*pbrInfo).n;
  let v = (*pbrInfo).v;
  let l = normalize(lightDirection);             // Vector from surface point to light
  let h = normalize(l + v);                      // Half vector between both l and v

  (*pbrInfo).NdotL = clamp(dot(n, l), 0.001, 1.0);
  (*pbrInfo).NdotH = clamp(dot(n, h), 0.0, 1.0);
  (*pbrInfo).LdotH = clamp(dot(l, h), 0.0, 1.0);
  (*pbrInfo).VdotH = clamp(dot(v, h), 0.0, 1.0);
}

fn PBRInfo_setPointLight(pbrInfo: ptr<function, PBRInfo>, pointLight: PointLight) {
  let light_direction = normalize(pointLight.position - pbr_vPosition);
  PBRInfo_setDirectionalLight(pbrInfo, light_direction);
}

fn calculateFinalColor(pbrInfo: PBRInfo, lightColor: vec3<f32>) -> vec3<f32> {
  // Calculate the shading terms for the microfacet specular shading model
  let F = specularReflection(pbrInfo);
  let G = geometricOcclusion(pbrInfo);
  let D = microfacetDistribution(pbrInfo);

  // Calculation of analytical lighting contribution
  let diffuseContrib = (1.0 - F) * diffuse(pbrInfo);
  let specContrib = F * G * D / (4.0 * pbrInfo.NdotL * pbrInfo.NdotV);
  // Obtain final intensity as reflectance (BRDF) scaled by the energy of the light (cosine law)
  return pbrInfo.NdotL * lightColor * (diffuseContrib + specContrib);
}

fn pbr_filterColor(colorUnused: vec4<f32>) -> vec4<f32> {
  // The albedo may be defined from a base texture or a flat color
  var baseColor: vec4<f32>;
  #ifdef HAS_BASECOLORMAP
  baseColor = SRGBtoLINEAR(textureSample(pbr_baseColorSampler, pbr_baseColorSampler, pbr_vUV)) * pbrMaterial.baseColorFactor;
  #else
  baseColor = pbrMaterial.baseColorFactor;
  #endif

  #ifdef ALPHA_CUTOFF
  if (baseColor.a < pbrMaterial.alphaCutoff) {
    discard;
  }
  #endif

  var color = vec3<f32>(0.0, 0.0, 0.0);

  if (pbrMaterial.unlit) {
    color = baseColor.rgb;
  } else {
    // Metallic and Roughness material properties are packed together
    // In glTF, these factors can be specified by fixed scalar values
    // or from a metallic-roughness map
    var perceptualRoughness = pbrMaterial.metallicRoughnessValues.y;
    var metallic = pbrMaterial.metallicRoughnessValues.x;
    #ifdef HAS_METALROUGHNESSMAP
    // Roughness is stored in the 'g' channel, metallic is stored in the 'b' channel.
    // This layout intentionally reserves the 'r' channel for (optional) occlusion map data
    let mrSample = textureSample(pbr_metallicRoughnessSampler, pbr_metallicRoughnessSampler, pbr_vUV);
    perceptualRoughness = mrSample.g * perceptualRoughness;
    metallic = mrSample.b * metallic;
    #endif
    perceptualRoughness = clamp(perceptualRoughness, c_MinRoughness, 1.0);
    metallic = clamp(metallic, 0.0, 1.0);
    // Roughness is authored as perceptual roughness; as is convention,
    // convert to material roughness by squaring the perceptual roughness [2].
    let alphaRoughness = perceptualRoughness * perceptualRoughness;

    let f0 = vec3<f32>(0.04);
    var diffuseColor = baseColor.rgb * (vec3<f32>(1.0) - f0);
    diffuseColor *= 1.0 - metallic;
    let specularColor = mix(f0, baseColor.rgb, metallic);

    // Compute reflectance.
    let reflectance = max(max(specularColor.r, specularColor.g), specularColor.b);

    // For typical incident reflectance range (between 4% to 100%) set the grazing
    // reflectance to 100% for typical fresnel effect.
    // For very low reflectance range on highly diffuse objects (below 4%),
    // incrementally reduce grazing reflectance to 0%.
    let reflectance90 = clamp(reflectance * 25.0, 0.0, 1.0);
    let specularEnvironmentR0 = specularColor;
    let specularEnvironmentR90 = vec3<f32>(1.0, 1.0, 1.0) * reflectance90;

    let n = getNormal();                          // normal at surface point
    let v = normalize(pbrProjection.camera - pbr_vPosition);  // Vector from surface point to camera

    let NdotV = clamp(abs(dot(n, v)), 0.001, 1.0);
    let reflection = -normalize(reflect(v, n));

    var pbrInfo = PBRInfo(
      0.0, // NdotL
      NdotV,
      0.0, // NdotH
      0.0, // LdotH
      0.0, // VdotH
      perceptualRoughness,
      metallic,
      specularEnvironmentR0,
      specularEnvironmentR90,
      alphaRoughness,
      diffuseColor,
      specularColor,
      n,
      v
    );

    #ifdef USE_LIGHTS
    // Apply ambient light
    PBRInfo_setAmbientLight(&pbrInfo);
    color += calculateFinalColor(pbrInfo, lighting.ambientColor);

    // Apply directional light
    for (var i = 0; i < lighting.directionalLightCount; i++) {
      if (i < lighting.directionalLightCount) {
        PBRInfo_setDirectionalLight(&pbrInfo, lighting_getDirectionalLight(i).direction);
        color += calculateFinalColor(pbrInfo, lighting_getDirectionalLight(i).color);
      }
    }

    // Apply point light
    for (var i = 0; i < lighting.pointLightCount; i++) {
      if (i < lighting.pointLightCount) {
        PBRInfo_setPointLight(&pbrInfo, lighting_getPointLight(i));
        let attenuation = getPointLightAttenuation(lighting_getPointLight(i), distance(lighting_getPointLight(i).position, pbr_vPosition));
        color += calculateFinalColor(pbrInfo, lighting_getPointLight(i).color / attenuation);
      }
    }
    #endif

    // Calculate lighting contribution from image based lighting source (IBL)
    #ifdef USE_IBL
    if (pbrMaterial.IBLenabled) {
      color += getIBLContribution(pbrInfo, n, reflection);
    }
    #endif

    // Apply optional PBR terms for additional (optional) shading
    #ifdef HAS_OCCLUSIONMAP
    if (pbrMaterial.occlusionMapEnabled) {
      let ao = textureSample(pbr_occlusionSampler, pbr_occlusionSampler, pbr_vUV).r;
      color = mix(color, color * ao, pbrMaterial.occlusionStrength);
    }
    #endif

    #ifdef HAS_EMISSIVEMAP
    if (pbrMaterial.emissiveMapEnabled) {
      let emissive = SRGBtoLINEAR(textureSample(pbr_emissiveSampler, pbr_emissiveSampler, pbr_vUV)).rgb * pbrMaterial.emissiveFactor;
      color += emissive;
    }
    #endif

    // This section uses mix to override final color for reference app visualization
    // of various parameters in the lighting equation.
    #ifdef PBR_DEBUG
    // TODO: Figure out how to debug multiple lights

    // color = mix(color, F, pbr_scaleFGDSpec.x);
    // color = mix(color, vec3(G), pbr_scaleFGDSpec.y);
    // color = mix(color, vec3(D), pbr_scaleFGDSpec.z);
    // color = mix(color, specContrib, pbr_scaleFGDSpec.w);

    // color = mix(color, diffuseContrib, pbr_scaleDiffBaseMR.x);
    color = mix(color, baseColor.rgb, pbrMaterial.scaleDiffBaseMR.y);
    color = mix(color, vec3<f32>(metallic), pbrMaterial.scaleDiffBaseMR.z);
    color = mix(color, vec3<f32>(perceptualRoughness), pbrMaterial.scaleDiffBaseMR.w);
    #endif
  }

  return vec4<f32>(pow(color, vec3<f32>(1.0 / 2.2)), baseColor.a);
}
`;var Ao=`uniform pbrProjectionUniforms {
  mat4 modelViewProjectionMatrix;
  mat4 modelMatrix;
  mat4 normalMatrix;
  vec3 camera;
} pbrProjection;
`,uo={name:"pbrProjection",vs:Ao,fs:Ao,getUniforms:t=>t,uniformTypes:{modelViewProjectionMatrix:"mat4x4<f32>",modelMatrix:"mat4x4<f32>",normalMatrix:"mat4x4<f32>",camera:"vec3<i32>"}};var Pe={props:{},uniforms:{},name:"pbrMaterial",dependencies:[se,uo],source:mo,vs:ho,fs:po,defines:{LIGHTING_FRAGMENT:!0,HAS_NORMALMAP:!1,HAS_EMISSIVEMAP:!1,HAS_OCCLUSIONMAP:!1,HAS_BASECOLORMAP:!1,HAS_METALROUGHNESSMAP:!1,ALPHA_CUTOFF:!1,USE_IBL:!1,PBR_DEBUG:!1},getUniforms:t=>t,uniformTypes:{unlit:"i32",baseColorMapEnabled:"i32",baseColorFactor:"vec4<f32>",normalMapEnabled:"i32",normalScale:"f32",emissiveMapEnabled:"i32",emissiveFactor:"vec3<f32>",metallicRoughnessValues:"vec2<f32>",metallicRoughnessMapEnabled:"i32",occlusionMapEnabled:"i32",occlusionStrength:"f32",alphaCutoffEnabled:"i32",alphaCutoff:"f32",IBLenabled:"i32",scaleIBLAmbient:"vec2<f32>",scaleDiffBaseMR:"vec4<f32>",scaleFGDSpec:"vec4<f32>"}};var de=F(pe(),1),$t=Math.PI/180,ft=new Float32Array(16),go=new Float32Array(12);function Bo(t,e,n){let r=e[0]*$t,o=e[1]*$t,s=e[2]*$t,i=Math.sin(s),a=Math.sin(r),c=Math.sin(o),f=Math.cos(s),l=Math.cos(r),h=Math.cos(o),p=n[0],m=n[1],A=n[2];t[0]=p*h*l,t[1]=p*c*l,t[2]=p*-a,t[3]=m*(-c*f+h*a*i),t[4]=m*(h*f+c*a*i),t[5]=m*l*i,t[6]=A*(c*i+h*a*f),t[7]=A*(-h*i+c*a*f),t[8]=A*l*f}function xo(t){return t[0]=t[0],t[1]=t[1],t[2]=t[2],t[3]=t[4],t[4]=t[5],t[5]=t[6],t[6]=t[8],t[7]=t[9],t[8]=t[10],t[9]=t[12],t[10]=t[13],t[11]=t[14],t.subarray(0,12)}var lt={size:12,accessor:["getOrientation","getScale","getTranslation","getTransformMatrix"],shaderAttributes:{instanceModelMatrixCol0:{size:3,elementOffset:0},instanceModelMatrixCol1:{size:3,elementOffset:3},instanceModelMatrixCol2:{size:3,elementOffset:6},instanceTranslation:{size:3,elementOffset:9}},update(t,{startRow:e,endRow:n}){let{data:r,getOrientation:o,getScale:s,getTranslation:i,getTransformMatrix:a}=this.props,c=Array.isArray(a),f=c&&a.length===16,l=Array.isArray(s),h=Array.isArray(o),p=Array.isArray(i),m=f||!c&&Boolean(a(r[0]));m?t.constant=f:t.constant=h&&l&&p;let A=t.value;if(t.constant){let u;m?(ft.set(a),u=xo(ft)):(u=go,Bo(u,o,s),u.set(i,9)),t.value=new Float32Array(u)}else{let u=e*t.size,{iterable:d,objectInfo:B}=(0,de.createIterable)(r,e,n);for(let E of d){B.index++;let g;if(m)ft.set(f?a:a(E,B)),g=xo(ft);else{g=go;let x=h?o:o(E,B),M=l?s:s(E,B);Bo(g,x,M),g.set(p?i:i(E,B),9)}A[u++]=g[0],A[u++]=g[1],A[u++]=g[2],A[u++]=g[3],A[u++]=g[4],A[u++]=g[5],A[u++]=g[6],A[u++]=g[7],A[u++]=g[8],A[u++]=g[9],A[u++]=g[10],A[u++]=g[11]}}}};function ht(t,e){return e===de.COORDINATE_SYSTEM.CARTESIAN||e===de.COORDINATE_SYSTEM.METER_OFFSETS||e===de.COORDINATE_SYSTEM.DEFAULT&&!t.isGeospatial}var Mo=`uniform simpleMeshUniforms {
  float sizeScale;
  bool composeModelMatrix;
  bool hasTexture;
  bool flatShading;
} simpleMesh;
`,Co={name:"simpleMesh",vs:Mo,fs:Mo,uniformTypes:{sizeScale:"f32",composeModelMatrix:"f32",hasTexture:"f32",flatShading:"f32"}};var Eo=`#version 300 es
#define SHADER_NAME simple-mesh-layer-vs
in vec3 positions;
in vec3 normals;
in vec3 colors;
in vec2 texCoords;
in vec3 instancePositions;
in vec3 instancePositions64Low;
in vec4 instanceColors;
in vec3 instancePickingColors;
in vec3 instanceModelMatrixCol0;
in vec3 instanceModelMatrixCol1;
in vec3 instanceModelMatrixCol2;
in vec3 instanceTranslation;
out vec2 vTexCoord;
out vec3 cameraPosition;
out vec3 normals_commonspace;
out vec4 position_commonspace;
out vec4 vColor;
void main(void) {
geometry.worldPosition = instancePositions;
geometry.uv = texCoords;
geometry.pickingColor = instancePickingColors;
vTexCoord = texCoords;
cameraPosition = project.cameraPosition;
vColor = vec4(colors * instanceColors.rgb, instanceColors.a);
mat3 instanceModelMatrix = mat3(instanceModelMatrixCol0, instanceModelMatrixCol1, instanceModelMatrixCol2);
vec3 pos = (instanceModelMatrix * positions) * simpleMesh.sizeScale + instanceTranslation;
if (simpleMesh.composeModelMatrix) {
DECKGL_FILTER_SIZE(pos, geometry);
normals_commonspace = project_normal(instanceModelMatrix * normals);
geometry.worldPosition += pos;
gl_Position = project_position_to_clipspace(pos + instancePositions, instancePositions64Low, vec3(0.0), position_commonspace);
geometry.position = position_commonspace;
}
else {
pos = project_size(pos);
DECKGL_FILTER_SIZE(pos, geometry);
gl_Position = project_position_to_clipspace(instancePositions, instancePositions64Low, pos, position_commonspace);
geometry.position = position_commonspace;
normals_commonspace = project_normal(instanceModelMatrix * normals);
}
geometry.normal = normals_commonspace;
DECKGL_FILTER_GL_POSITION(gl_Position, geometry);
DECKGL_FILTER_COLOR(vColor, geometry);
}
`;var bo=`#version 300 es
#define SHADER_NAME simple-mesh-layer-fs
precision highp float;
uniform sampler2D sampler;
in vec2 vTexCoord;
in vec3 cameraPosition;
in vec3 normals_commonspace;
in vec4 position_commonspace;
in vec4 vColor;
out vec4 fragColor;
void main(void) {
geometry.uv = vTexCoord;
vec3 normal;
if (simpleMesh.flatShading) {
normal = normalize(cross(dFdx(position_commonspace.xyz), dFdy(position_commonspace.xyz)));
} else {
normal = normals_commonspace;
}
vec4 color = simpleMesh.hasTexture ? texture(sampler, vTexCoord) : vColor;
DECKGL_FILTER_COLOR(color, geometry);
vec3 lightColor = lighting_getLightColor(color.rgb, cameraPosition, position_commonspace.xyz, normal);
fragColor = vec4(lightColor, color.a * layer.opacity);
}
`;function _o(t){switch(t.constructor){case Int8Array:return"int8";case Uint8Array:case Uint8ClampedArray:return"uint8";case Int16Array:return"int16";case Uint16Array:return"uint16";case Int32Array:return"int32";case Uint32Array:return"uint32";case Float32Array:return"float32";case Float64Array:return"float64";default:return"null"}}function we(t){let e=1/0,n=1/0,r=1/0,o=-1/0,s=-1/0,i=-1/0,a=t.POSITION?t.POSITION.value:[],c=a&&a.length;for(let f=0;f<c;f+=3){let l=a[f],h=a[f+1],p=a[f+2];e=l<e?l:e,n=h<n?h:n,r=p<r?p:r,o=l>o?l:o,s=h>s?h:s,i=p>i?p:i}return[[e,n,r],[o,s,i]]}function en(t,e,n){let r=_o(e.value),o=n||yo(e);return{name:t,type:{type:"fixed-size-list",listSize:e.size,children:[{name:"value",type:r}]},nullable:!1,metadata:o}}function yo(t){let e={};return"byteOffset"in t&&(e.byteOffset=t.byteOffset.toString(10)),"byteStride"in t&&(e.byteStride=t.byteStride.toString(10)),"normalized"in t&&(e.normalized=t.normalized.toString()),e}function tn(t){let e=t.positions||t.POSITION;te.log.assert(e,'no "postions" or "POSITION" attribute in mesh');let n=e.value.length/e.size,r=t.COLOR_0||t.colors;r||(r={size:3,value:new Float32Array(n*3).fill(1)});let o=t.NORMAL||t.normals;o||(o={size:3,value:new Float32Array(n*3).fill(0)});let s=t.TEXCOORD_0||t.texCoords;return s||(s={size:2,value:new Float32Array(n*2).fill(0)}),{positions:e,colors:r,normals:o,texCoords:s}}function To(t){return t instanceof ge.Geometry?(t.attributes=tn(t.attributes),t):t.attributes?new ge.Geometry({...t,topology:"triangle-list",attributes:tn(t.attributes)}):new ge.Geometry({topology:"triangle-list",attributes:tn(t)})}var ra=[0,0,0,255],oa={mesh:{type:"object",value:null,async:!0},texture:{type:"image",value:null,async:!0},sizeScale:{type:"number",value:1,min:0},_instanced:!0,wireframe:!1,material:!0,getPosition:{type:"accessor",value:t=>t.position},getColor:{type:"accessor",value:ra},getOrientation:{type:"accessor",value:[0,0,0]},getScale:{type:"accessor",value:[1,1,1]},getTranslation:{type:"accessor",value:[0,0,0]},getTransformMatrix:{type:"accessor",value:[]},textureParameters:{type:"object",ignore:!0,value:null}},Ne=class extends te.Layer{getShaders(){return super.getShaders({vs:Eo,fs:bo,modules:[te.project32,ct,te.picking,Co]})}getBounds(){if(this.props._instanced)return super.getBounds();let e=this.state.positionBounds;if(e)return e;let{mesh:n}=this.props;if(!n)return null;if(e=n.header?.boundingBox,!e){let{attributes:r}=To(n);r.POSITION=r.POSITION||r.positions,e=we(r)}return this.state.positionBounds=e,e}initializeState(){this.getAttributeManager().addInstanced({instancePositions:{transition:!0,type:"float64",fp64:this.use64bitPositions(),size:3,accessor:"getPosition"},instanceColors:{type:"unorm8",transition:!0,size:this.props.colorFormat.length,accessor:"getColor",defaultValue:[0,0,0,255]},instanceModelMatrix:lt}),this.setState({emptyTexture:this.context.device.createTexture({data:new Uint8Array(4),width:1,height:1})})}updateState(e){super.updateState(e);let{props:n,oldProps:r,changeFlags:o}=e;if(n.mesh!==r.mesh||o.extensionsChanged){if(this.state.positionBounds=null,this.state.model?.destroy(),n.mesh){this.state.model=this.getModel(n.mesh);let s=n.mesh.attributes||n.mesh;this.setState({hasNormals:Boolean(s.NORMAL||s.normals)})}this.getAttributeManager().invalidateAll()}n.texture!==r.texture&&n.texture instanceof Ro.Texture&&this.setTexture(n.texture),this.state.model&&this.state.model.setTopology(this.props.wireframe?"line-strip":"triangle-list")}finalizeState(e){super.finalizeState(e),this.state.emptyTexture.delete()}draw({uniforms:e}){let{model:n}=this.state;if(!n)return;let{viewport:r,renderPass:o}=this.context,{sizeScale:s,coordinateSystem:i,_instanced:a}=this.props,c={sizeScale:s,composeModelMatrix:!a||ht(r,i),flatShading:!this.state.hasNormals};n.shaderInputs.setProps({simpleMesh:c}),n.draw(o)}get isLoaded(){return Boolean(this.state?.model&&super.isLoaded)}getModel(e){let n=new ge.Model(this.context.device,{...this.getShaders(),id:this.props.id,bufferLayout:this.getAttributeManager().getBufferLayouts(),geometry:To(e),isInstanced:!0}),{texture:r}=this.props,{emptyTexture:o}=this.state,s={sampler:r||o,hasTexture:Boolean(r)};return n.shaderInputs.setProps({simpleMesh:s}),n}setTexture(e){let{emptyTexture:n,model:r}=this.state;if(r){let o={sampler:e||n,hasTexture:Boolean(e)};r.shaderInputs.setProps({simpleMesh:o})}}};Ne.defaultProps=oa;Ne.layerName="SimpleMeshLayer";var Io=Ne;var Q=F(pe(),1);var he=F(Fe(),1);var So="4.2.1";async function Ue(t,e,n,r){return r._parse(t,e,n,r)}function U(t,e){if(!t)throw new Error(e||"loader assertion failed.")}var j={self:typeof self<"u"&&self,window:typeof window<"u"&&window,global:typeof global<"u"&&global,document:typeof document<"u"&&document},sa=j.self||j.window||j.global||{},ia=j.window||j.self||j.global||{},aa=j.global||j.self||j.window||{},ca=j.document||{};var He=Boolean(typeof process!="object"||String(process)!=="[object process]"||process.browser);var Lo=typeof process<"u"&&process.version&&/v([0-9]*)/.exec(process.version),fa=Lo&&parseFloat(Lo[1])||0;function nn(t){globalThis.loaders||={},globalThis.loaders.modules||={},Object.assign(globalThis.loaders.modules,t)}function rn(t){return globalThis.loaders?.modules?.[t]||null}function la(){return globalThis._loadersgl_?.version||(globalThis._loadersgl_=globalThis._loadersgl_||{},globalThis._loadersgl_.version="4.2.1"),globalThis._loadersgl_.version}var Fo=la();function Do(t,e){if(!t)throw new Error(e||"loaders.gl assertion failed.")}var k={self:typeof self<"u"&&self,window:typeof window<"u"&&window,global:typeof global<"u"&&global,document:typeof document<"u"&&document},qp=k.self||k.window||k.global||{},Zp=k.window||k.self||k.global||{},$p=k.global||k.self||k.window||{},em=k.document||{};var Be=typeof process!="object"||String(process)!=="[object process]"||process.browser,pt=typeof importScripts=="function",tm=typeof window<"u"&&typeof window.orientation<"u",vo=typeof process<"u"&&process.version&&/v([0-9]*)/.exec(process.version),nm=vo&&parseFloat(vo[1])||0;var on={};async function z(t,e=null,n={},r=null){return e&&(t=Go(t,e,n,r)),on[t]=on[t]||ha(t),await on[t]}function Go(t,e,n={},r=null){if(!n.useLocalLibraries&&t.startsWith("http"))return t;r=r||t;let o=n.modules||{};return o[r]?o[r]:Be?n.CDN?(Do(n.CDN.startsWith("http")),`${n.CDN}/${e}@${Fo}/dist/libs/${r}`):pt?`../src/libs/${r}`:`modules/${e}/src/libs/${r}`:`modules/${e}/dist/libs/${r}`}async function ha(t){if(t.endsWith("wasm"))return await ma(t);if(!Be)try{let{requireFromFile:n}=globalThis.loaders||{};return await n?.(t)}catch(n){return console.error(n),null}if(pt)return importScripts(t);let e=await Aa(t);return pa(e,t)}function pa(t,e){if(!Be){let{requireFromString:r}=globalThis.loaders||{};return r?.(t,e)}if(pt)return eval.call(globalThis,t),null;let n=document.createElement("script");n.id=e;try{n.appendChild(document.createTextNode(t))}catch{n.text=t}return document.body.appendChild(n),null}async function ma(t){let{readFileAsArrayBuffer:e}=globalThis.loaders||{};return Be||!e||t.startsWith("http")?await(await fetch(t)).arrayBuffer():await e(t)}async function Aa(t){let{readFileAsText:e}=globalThis.loaders||{};return Be||!e||t.startsWith("http")?await(await fetch(t)).text():await e(t)}function Po(t,e=5){return typeof t=="string"?t.slice(0,e):ArrayBuffer.isView(t)?Oo(t.buffer,t.byteOffset,e):t instanceof ArrayBuffer?Oo(t,0,e):""}function Oo(t,e,n){if(t.byteLength<=e+n)return"";let r=new DataView(t),o="";for(let s=0;s<n;s++)o+=String.fromCharCode(r.getUint8(e+s));return o}function sn(t){try{return JSON.parse(t)}catch{throw new Error(`Failed to parse JSON from data starting with "${Po(t)}"`)}}function Je(t,e,n){let r=n!==void 0?new Uint8Array(t).subarray(e,e+n):new Uint8Array(t).subarray(e);return new Uint8Array(r).buffer}function ne(t,e){return U(t>=0),U(e>0),t+(e-1)&~(e-1)}function an(t,e,n){let r;if(t instanceof ArrayBuffer)r=new Uint8Array(t);else{let o=t.byteOffset,s=t.byteLength;r=new Uint8Array(t.buffer||t.arrayBuffer,o,s)}return e.set(r,n),n+ne(r.byteLength,4)}var mt={TRANSCODER:"basis_transcoder.js",TRANSCODER_WASM:"basis_transcoder.wasm",ENCODER:"basis_encoder.js",ENCODER_WASM:"basis_encoder.wasm"},wo;async function fn(t){nn(t.modules);let e=rn("basis");return e||(wo||=ua(t),await wo)}async function ua(t){let e=null,n=null;return[e,n]=await Promise.all([await z(mt.TRANSCODER,"textures",t),await z(mt.TRANSCODER_WASM,"textures",t)]),e=e||globalThis.BASIS,await da(e,n)}function da(t,e){let n={};return e&&(n.wasmBinary=e),new Promise(r=>{t(n).then(o=>{let{BasisFile:s,initializeBasis:i}=o;i(),r({BasisFile:s})})})}var cn;async function ln(t){let e=t.modules||{};return e.basisEncoder?e.basisEncoder:(cn=cn||ga(t),await cn)}async function ga(t){let e=null,n=null;return[e,n]=await Promise.all([await z(mt.ENCODER,"textures",t),await z(mt.ENCODER_WASM,"textures",t)]),e=e||globalThis.BASIS,await Ba(e,n)}function Ba(t,e){let n={};return e&&(n.wasmBinary=e),new Promise(r=>{t(n).then(o=>{let{BasisFile:s,KTX2File:i,initializeBasis:a,BasisEncoder:c}=o;a(),r({BasisFile:s,KTX2File:i,BasisEncoder:c})})})}var ie={COMPRESSED_RGB_S3TC_DXT1_EXT:33776,COMPRESSED_RGBA_S3TC_DXT1_EXT:33777,COMPRESSED_RGBA_S3TC_DXT3_EXT:33778,COMPRESSED_RGBA_S3TC_DXT5_EXT:33779,COMPRESSED_R11_EAC:37488,COMPRESSED_SIGNED_R11_EAC:37489,COMPRESSED_RG11_EAC:37490,COMPRESSED_SIGNED_RG11_EAC:37491,COMPRESSED_RGB8_ETC2:37492,COMPRESSED_RGBA8_ETC2_EAC:37493,COMPRESSED_SRGB8_ETC2:37494,COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:37495,COMPRESSED_RGB8_PUNCHTHROUGH_ALPHA1_ETC2:37496,COMPRESSED_SRGB8_PUNCHTHROUGH_ALPHA1_ETC2:37497,COMPRESSED_RGB_PVRTC_4BPPV1_IMG:35840,COMPRESSED_RGBA_PVRTC_4BPPV1_IMG:35842,COMPRESSED_RGB_PVRTC_2BPPV1_IMG:35841,COMPRESSED_RGBA_PVRTC_2BPPV1_IMG:35843,COMPRESSED_RGB_ETC1_WEBGL:36196,COMPRESSED_RGB_ATC_WEBGL:35986,COMPRESSED_RGBA_ATC_EXPLICIT_ALPHA_WEBGL:35987,COMPRESSED_RGBA_ATC_INTERPOLATED_ALPHA_WEBGL:34798,COMPRESSED_RGBA_ASTC_4X4_KHR:37808,COMPRESSED_RGBA_ASTC_5X4_KHR:37809,COMPRESSED_RGBA_ASTC_5X5_KHR:37810,COMPRESSED_RGBA_ASTC_6X5_KHR:37811,COMPRESSED_RGBA_ASTC_6X6_KHR:37812,COMPRESSED_RGBA_ASTC_8X5_KHR:37813,COMPRESSED_RGBA_ASTC_8X6_KHR:37814,COMPRESSED_RGBA_ASTC_8X8_KHR:37815,COMPRESSED_RGBA_ASTC_10X5_KHR:37816,COMPRESSED_RGBA_ASTC_10X6_KHR:37817,COMPRESSED_RGBA_ASTC_10X8_KHR:37818,COMPRESSED_RGBA_ASTC_10X10_KHR:37819,COMPRESSED_RGBA_ASTC_12X10_KHR:37820,COMPRESSED_RGBA_ASTC_12X12_KHR:37821,COMPRESSED_SRGB8_ALPHA8_ASTC_4X4_KHR:37840,COMPRESSED_SRGB8_ALPHA8_ASTC_5X4_KHR:37841,COMPRESSED_SRGB8_ALPHA8_ASTC_5X5_KHR:37842,COMPRESSED_SRGB8_ALPHA8_ASTC_6X5_KHR:37843,COMPRESSED_SRGB8_ALPHA8_ASTC_6X6_KHR:37844,COMPRESSED_SRGB8_ALPHA8_ASTC_8X5_KHR:37845,COMPRESSED_SRGB8_ALPHA8_ASTC_8X6_KHR:37846,COMPRESSED_SRGB8_ALPHA8_ASTC_8X8_KHR:37847,COMPRESSED_SRGB8_ALPHA8_ASTC_10X5_KHR:37848,COMPRESSED_SRGB8_ALPHA8_ASTC_10X6_KHR:37849,COMPRESSED_SRGB8_ALPHA8_ASTC_10X8_KHR:37850,COMPRESSED_SRGB8_ALPHA8_ASTC_10X10_KHR:37851,COMPRESSED_SRGB8_ALPHA8_ASTC_12X10_KHR:37852,COMPRESSED_SRGB8_ALPHA8_ASTC_12X12_KHR:37853,COMPRESSED_RED_RGTC1_EXT:36283,COMPRESSED_SIGNED_RED_RGTC1_EXT:36284,COMPRESSED_RED_GREEN_RGTC2_EXT:36285,COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT:36286,COMPRESSED_SRGB_S3TC_DXT1_EXT:35916,COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT:35917,COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT:35918,COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT:35919};var xa=["","WEBKIT_","MOZ_"],No={WEBGL_compressed_texture_s3tc:"dxt",WEBGL_compressed_texture_s3tc_srgb:"dxt-srgb",WEBGL_compressed_texture_etc1:"etc1",WEBGL_compressed_texture_etc:"etc2",WEBGL_compressed_texture_pvrtc:"pvrtc",WEBGL_compressed_texture_atc:"atc",WEBGL_compressed_texture_astc:"astc",EXT_texture_compression_rgtc:"rgtc"},At=null;function Uo(t){if(!At){t=t||Ma()||void 0,At=new Set;for(let e of xa)for(let n in No)if(t&&t.getExtension(`${e}${n}`)){let r=No[n];At.add(r)}}return At}function Ma(){try{return document.createElement("canvas").getContext("webgl")}catch{return null}}var Hm=new Uint8Array([0]);var Ho,Jo,Ko,Vo,Xo,jo,ko,zo;(function(t){t[t.NONE=0]="NONE",t[t.BASISLZ=1]="BASISLZ",t[t.ZSTD=2]="ZSTD",t[t.ZLIB=3]="ZLIB"})(Ho||(Ho={})),function(t){t[t.BASICFORMAT=0]="BASICFORMAT"}(Jo||(Jo={})),function(t){t[t.UNSPECIFIED=0]="UNSPECIFIED",t[t.ETC1S=163]="ETC1S",t[t.UASTC=166]="UASTC"}(Ko||(Ko={})),function(t){t[t.UNSPECIFIED=0]="UNSPECIFIED",t[t.SRGB=1]="SRGB"}(Vo||(Vo={})),function(t){t[t.UNSPECIFIED=0]="UNSPECIFIED",t[t.LINEAR=1]="LINEAR",t[t.SRGB=2]="SRGB",t[t.ITU=3]="ITU",t[t.NTSC=4]="NTSC",t[t.SLOG=5]="SLOG",t[t.SLOG2=6]="SLOG2"}(Xo||(Xo={})),function(t){t[t.ALPHA_STRAIGHT=0]="ALPHA_STRAIGHT",t[t.ALPHA_PREMULTIPLIED=1]="ALPHA_PREMULTIPLIED"}(jo||(jo={})),function(t){t[t.RGB=0]="RGB",t[t.RRR=3]="RRR",t[t.GGG=4]="GGG",t[t.AAA=15]="AAA"}(ko||(ko={})),function(t){t[t.RGB=0]="RGB",t[t.RGBA=3]="RGBA",t[t.RRR=4]="RRR",t[t.RRRG=5]="RRRG"}(zo||(zo={}));var w=[171,75,84,88,32,50,48,187,13,10,26,10];function Yo(t){let e=new Uint8Array(t);return!(e.byteLength<w.length||e[0]!==w[0]||e[1]!==w[1]||e[2]!==w[2]||e[3]!==w[3]||e[4]!==w[4]||e[5]!==w[5]||e[6]!==w[6]||e[7]!==w[7]||e[8]!==w[8]||e[9]!==w[9]||e[10]!==w[10]||e[11]!==w[11])}var Ca={etc1:{basisFormat:0,compressed:!0,format:ie.COMPRESSED_RGB_ETC1_WEBGL},etc2:{basisFormat:1,compressed:!0},bc1:{basisFormat:2,compressed:!0,format:ie.COMPRESSED_RGB_S3TC_DXT1_EXT},bc3:{basisFormat:3,compressed:!0,format:ie.COMPRESSED_RGBA_S3TC_DXT5_EXT},bc4:{basisFormat:4,compressed:!0},bc5:{basisFormat:5,compressed:!0},"bc7-m6-opaque-only":{basisFormat:6,compressed:!0},"bc7-m5":{basisFormat:7,compressed:!0},"pvrtc1-4-rgb":{basisFormat:8,compressed:!0,format:ie.COMPRESSED_RGB_PVRTC_4BPPV1_IMG},"pvrtc1-4-rgba":{basisFormat:9,compressed:!0,format:ie.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG},"astc-4x4":{basisFormat:10,compressed:!0,format:ie.COMPRESSED_RGBA_ASTC_4X4_KHR},"atc-rgb":{basisFormat:11,compressed:!0},"atc-rgba-interpolated-alpha":{basisFormat:12,compressed:!0},rgba32:{basisFormat:13,compressed:!1},rgb565:{basisFormat:14,compressed:!1},bgr565:{basisFormat:15,compressed:!1},rgba4444:{basisFormat:16,compressed:!1}};async function pn(t,e){if(e.basis.containerFormat==="auto"){if(Yo(t)){let r=await ln(e);return Qo(r.KTX2File,t,e)}let{BasisFile:n}=await fn(e);return hn(n,t,e)}switch(e.basis.module){case"encoder":let n=await ln(e);switch(e.basis.containerFormat){case"ktx2":return Qo(n.KTX2File,t,e);case"basis":default:return hn(n.BasisFile,t,e)}case"transcoder":default:let{BasisFile:r}=await fn(e);return hn(r,t,e)}}function hn(t,e,n){let r=new t(new Uint8Array(e));try{if(!r.startTranscoding())throw new Error("Failed to start basis transcoding");let o=r.getNumImages(),s=[];for(let i=0;i<o;i++){let a=r.getNumLevels(i),c=[];for(let f=0;f<a;f++)c.push(Ea(r,i,f,n));s.push(c)}return s}finally{r.close(),r.delete()}}function Ea(t,e,n,r){let o=t.getImageWidth(e,n),s=t.getImageHeight(e,n),i=t.getHasAlpha(),{compressed:a,format:c,basisFormat:f}=Wo(r,i),l=t.getImageTranscodedSizeInBytes(e,n,f),h=new Uint8Array(l);if(!t.transcodeImage(h,e,n,f,0,0))throw new Error("failed to start Basis transcoding");return{width:o,height:s,data:h,compressed:a,format:c,hasAlpha:i}}function Qo(t,e,n){let r=new t(new Uint8Array(e));try{if(!r.startTranscoding())throw new Error("failed to start KTX2 transcoding");let o=r.getLevels(),s=[];for(let i=0;i<o;i++){s.push(ba(r,i,n));break}return[s]}finally{r.close(),r.delete()}}function ba(t,e,n){let{alphaFlag:r,height:o,width:s}=t.getImageLevelInfo(e,0,0),{compressed:i,format:a,basisFormat:c}=Wo(n,r),f=t.getImageTranscodedSizeInBytes(e,0,0,c),l=new Uint8Array(f);if(!t.transcodeImage(l,e,0,0,c,0,-1,-1))throw new Error("Failed to transcode KTX2 image");return{width:s,height:o,data:l,compressed:i,levelSize:f,hasAlpha:r,format:a}}function Wo(t,e){let n=t&&t.basis&&t.basis.format;return n==="auto"&&(n=ut()),typeof n=="object"&&(n=e?n.alpha:n.noAlpha),n=n.toLowerCase(),Ca[n]}function ut(){let t=Uo();return t.has("astc")?"astc-4x4":t.has("dxt")?{alpha:"bc3",noAlpha:"bc1"}:t.has("pvrtc")?{alpha:"pvrtc1-4-rgba",noAlpha:"pvrtc1-4-rgb"}:t.has("etc1")?"etc1":t.has("etc2")?"etc2":"rgb565"}var qo={dataType:null,batchType:null,name:"Basis",id:"basis",module:"textures",version:So,worker:!0,extensions:["basis","ktx2"],mimeTypes:["application/octet-stream","image/ktx2"],tests:["sB"],binary:!0,options:{basis:{format:"auto",libraryPath:"libs/",containerFormat:"auto",module:"transcoder"}}},mn={...qo,parse:pn};var Zo="4.2.1";var _a=globalThis.loaders?.parseImageNode,An=typeof Image<"u",un=typeof ImageBitmap<"u",ya=Boolean(_a),dn=He?!0:ya;function $o(t){switch(t){case"auto":return un||An||dn;case"imagebitmap":return un;case"image":return An;case"data":return dn;default:throw new Error(`@loaders.gl/images: image ${t} not supported in this environment`)}}function es(){if(un)return"imagebitmap";if(An)return"image";if(dn)return"data";throw new Error("Install '@loaders.gl/polyfills' to parse images under Node.js")}function ts(t){let e=Ta(t);if(!e)throw new Error("Not an image");return e}function Ke(t){switch(ts(t)){case"data":return t;case"image":case"imagebitmap":let e=document.createElement("canvas"),n=e.getContext("2d");if(!n)throw new Error("getImageData");return e.width=t.width,e.height=t.height,n.drawImage(t,0,0),n.getImageData(0,0,t.width,t.height);default:throw new Error("getImageData")}}function Ta(t){return typeof ImageBitmap<"u"&&t instanceof ImageBitmap?"imagebitmap":typeof Image<"u"&&t instanceof Image?"image":t&&typeof t=="object"&&t.data&&t.width&&t.height?"data":null}var Ra=/^data:image\/svg\+xml/,Ia=/\.svg((\?|#).*)?$/;function dt(t){return t&&(Ra.test(t)||Ia.test(t))}function ns(t,e){if(dt(e)){let r=new TextDecoder().decode(t);try{typeof unescape=="function"&&typeof encodeURIComponent=="function"&&(r=unescape(encodeURIComponent(r)))}catch(s){throw new Error(s.message)}return`data:image/svg+xml;base64,${btoa(r)}`}return gn(t,e)}function gn(t,e){if(dt(e))throw new Error("SVG cannot be parsed directly to imagebitmap");return new Blob([new Uint8Array(t)])}async function gt(t,e,n){let r=ns(t,n),o=self.URL||self.webkitURL,s=typeof r!="string"&&o.createObjectURL(r);try{return await Sa(s||r,e)}finally{s&&o.revokeObjectURL(s)}}async function Sa(t,e){let n=new Image;return n.src=t,e.image&&e.image.decode&&n.decode?(await n.decode(),n):await new Promise((r,o)=>{try{n.onload=()=>r(n),n.onerror=s=>{let i=s instanceof Error?s.message:"error";o(new Error(i))}}catch(s){o(s)}})}var La={},rs=!0;async function os(t,e,n){let r;dt(n)?r=await gt(t,e,n):r=gn(t,n);let o=e&&e.imagebitmap;return await Fa(r,o)}async function Fa(t,e=null){if((Da(e)||!rs)&&(e=null),e)try{return await createImageBitmap(t,e)}catch(n){console.warn(n),rs=!1}return await createImageBitmap(t)}function Da(t){for(let e in t||La)return!1;return!0}function ss(t){return!Pa(t,"ftyp",4)||!(t[8]&96)?null:va(t)}function va(t){switch(Ga(t,8,12).replace("\0"," ").trim()){case"avif":case"avis":return{extension:"avif",mimeType:"image/avif"};default:return null}}function Ga(t,e,n){return String.fromCharCode(...t.slice(e,n))}function Oa(t){return[...t].map(e=>e.charCodeAt(0))}function Pa(t,e,n=0){let r=Oa(e);for(let o=0;o<r.length;++o)if(r[o]!==t[o+n])return!1;return!0}var Y=!1,Ve=!0;function ae(t){let e=Xe(t);return Na(e)||Ja(e)||Ua(e)||Ha(e)||wa(e)}function wa(t){let e=new Uint8Array(t instanceof DataView?t.buffer:t),n=ss(e);return n?{mimeType:n.mimeType,width:0,height:0}:null}function Na(t){let e=Xe(t);return e.byteLength>=24&&e.getUint32(0,Y)===2303741511?{mimeType:"image/png",width:e.getUint32(16,Y),height:e.getUint32(20,Y)}:null}function Ua(t){let e=Xe(t);return e.byteLength>=10&&e.getUint32(0,Y)===1195984440?{mimeType:"image/gif",width:e.getUint16(6,Ve),height:e.getUint16(8,Ve)}:null}function Ha(t){let e=Xe(t);return e.byteLength>=14&&e.getUint16(0,Y)===16973&&e.getUint32(2,Ve)===e.byteLength?{mimeType:"image/bmp",width:e.getUint32(18,Ve),height:e.getUint32(22,Ve)}:null}function Ja(t){let e=Xe(t);if(!(e.byteLength>=3&&e.getUint16(0,Y)===65496&&e.getUint8(2)===255))return null;let{tableMarkers:r,sofMarkers:o}=Ka(),s=2;for(;s+9<e.byteLength;){let i=e.getUint16(s,Y);if(o.has(i))return{mimeType:"image/jpeg",height:e.getUint16(s+5,Y),width:e.getUint16(s+7,Y)};if(!r.has(i))return null;s+=2,s+=e.getUint16(s,Y)}return null}function Ka(){let t=new Set([65499,65476,65484,65501,65534]);for(let n=65504;n<65520;++n)t.add(n);return{tableMarkers:t,sofMarkers:new Set([65472,65473,65474,65475,65477,65478,65479,65481,65482,65483,65485,65486,65487,65502])}}function Xe(t){if(t instanceof DataView)return t;if(ArrayBuffer.isView(t))return new DataView(t.buffer);if(t instanceof ArrayBuffer)return new DataView(t);throw new Error("toDataView")}async function is(t,e){let{mimeType:n}=ae(t)||{},r=globalThis.loaders?.parseImageNode;return U(r),await r(t,n)}async function as(t,e,n){e=e||{};let o=(e.image||{}).type||"auto",{url:s}=n||{},i=Va(o),a;switch(i){case"imagebitmap":a=await os(t,e,s);break;case"image":a=await gt(t,e,s);break;case"data":a=await is(t,e);break;default:U(!1)}return o==="data"&&(a=Ke(a)),a}function Va(t){switch(t){case"auto":case"data":return es();default:return $o(t),t}}var Xa=["png","jpg","jpeg","gif","webp","bmp","ico","svg","avif"],ja=["image/png","image/jpeg","image/gif","image/webp","image/avif","image/bmp","image/vnd.microsoft.icon","image/svg+xml"],ka={image:{type:"auto",decode:!0}},Bn={dataType:null,batchType:null,id:"image",module:"images",name:"Images",version:Zo,mimeTypes:ja,extensions:Xa,parse:as,tests:[t=>Boolean(ae(new DataView(t)))],options:ka};var xn={};function Mn(t){if(xn[t]===void 0){let e=He?Ya(t):za(t);xn[t]=e}return xn[t]}function za(t){let e=["image/png","image/jpeg","image/gif"],n=globalThis.loaders?.imageFormatsNode||e,r=globalThis.loaders?.parseImageNode;return Boolean(r)&&n.includes(t)}function Ya(t){switch(t){case"image/avif":case"image/webp":return Qa(t);default:return!0}}function Qa(t){try{return document.createElement("canvas").toDataURL(t).indexOf(`data:${t}`)===0}catch{return!1}}var ls=F(oe(),1);function fs(t){return{addressModeU:cs(t.wrapS),addressModeV:cs(t.wrapT),magFilter:Wa(t.magFilter),...qa(t.minFilter)}}function cs(t){switch(t){case 33071:return"clamp-to-edge";case 10497:return"repeat";case 33648:return"mirror-repeat";default:return}}function Wa(t){switch(t){case 9728:return"nearest";case 9729:return"linear";default:return}}function qa(t){switch(t){case 9728:return{minFilter:"nearest"};case 9729:return{minFilter:"linear"};case 9984:return{minFilter:"nearest",mipmapFilter:"nearest"};case 9985:return{minFilter:"linear",mipmapFilter:"nearest"};case 9986:return{minFilter:"nearest",mipmapFilter:"linear"};case 9987:return{minFilter:"linear",mipmapFilter:"linear"};default:return{}}}function hs(t,e,n,r){let o={defines:{MANUAL_SRGB:!0,SRGB_FAST_APPROXIMATION:!0},bindings:{},uniforms:{camera:[0,0,0],metallicRoughnessValues:[1,1]},parameters:{},glParameters:{},generatedTextures:[]};o.defines.USE_TEX_LOD=!0;let{imageBasedLightingEnvironment:s}=r;return s&&(o.bindings.pbr_diffuseEnvSampler=s.diffuseEnvSampler.texture,o.bindings.pbr_specularEnvSampler=s.specularEnvSampler.texture,o.bindings.pbr_BrdfLUT=s.brdfLutTexture.texture,o.uniforms.scaleIBLAmbient=[1,1]),r?.pbrDebug&&(o.defines.PBR_DEBUG=!0,o.uniforms.scaleDiffBaseMR=[0,0,0,0],o.uniforms.scaleFGDSpec=[0,0,0,0]),n.NORMAL&&(o.defines.HAS_NORMALS=!0),n.TANGENT&&r?.useTangents&&(o.defines.HAS_TANGENTS=!0),n.TEXCOORD_0&&(o.defines.HAS_UV=!0),r?.imageBasedLightingEnvironment&&(o.defines.USE_IBL=!0),r?.lights&&(o.defines.USE_LIGHTS=!0),e&&Za(t,e,o),o}function Za(t,e,n){if(n.uniforms.unlit=Boolean(e.unlit),e.pbrMetallicRoughness&&$a(t,e.pbrMetallicRoughness,n),e.normalTexture){je(t,e.normalTexture,"pbr_normalSampler","HAS_NORMALMAP",n);let{scale:r=1}=e.normalTexture;n.uniforms.normalScale=r}if(e.occlusionTexture){je(t,e.occlusionTexture,"pbr_occlusionSampler","HAS_OCCLUSIONMAP",n);let{strength:r=1}=e.occlusionTexture;n.uniforms.occlusionStrength=r}switch(e.emissiveTexture&&(je(t,e.emissiveTexture,"pbr_emissiveSampler","HAS_EMISSIVEMAP",n),n.uniforms.emissiveFactor=e.emissiveFactor||[0,0,0]),e.alphaMode||"MASK"){case"MASK":let{alphaCutoff:r=.5}=e;n.defines.ALPHA_CUTOFF=!0,n.uniforms.alphaCutoff=r;break;case"BLEND":ls.log.warn("glTF BLEND alphaMode might not work well because it requires mesh sorting")(),n.parameters.blend=!0,n.parameters.blendColorOperation="add",n.parameters.blendColorSrcFactor="src-alpha",n.parameters.blendColorDstFactor="one-minus-src-alpha",n.parameters.blendAlphaOperation="add",n.parameters.blendAlphaSrcFactor="one",n.parameters.blendAlphaDstFactor="one-minus-src-alpha",n.glParameters.blend=!0,n.glParameters.blendEquation=32774,n.glParameters.blendFunc=[770,771,1,771];break}}function $a(t,e,n){e.baseColorTexture&&je(t,e.baseColorTexture,"pbr_baseColorSampler","HAS_BASECOLORMAP",n),n.uniforms.baseColorFactor=e.baseColorFactor||[1,1,1,1],e.metallicRoughnessTexture&&je(t,e.metallicRoughnessTexture,"pbr_metallicRoughnessSampler","HAS_METALROUGHNESSMAP",n);let{metallicFactor:r=1,roughnessFactor:o=1}=e;n.uniforms.metallicRoughnessValues=[r,o]}function je(t,e,n,r,o){let s=e.texture.source.image,i;s.compressed?i=s:i={data:s};let a={wrapS:10497,wrapT:10497,...e?.texture?.sampler},c=t.createTexture({id:e.uniformName||e.id,sampler:fs(a),...i});o.bindings[n]=c,r&&(o.defines[r]=!0),o.generatedTextures.push(c)}var xe=F(Fe(),1);var ce;(function(t){t[t.POINTS=0]="POINTS",t[t.LINES=1]="LINES",t[t.LINE_LOOP=2]="LINE_LOOP",t[t.LINE_STRIP=3]="LINE_STRIP",t[t.TRIANGLES=4]="TRIANGLES",t[t.TRIANGLE_STRIP=5]="TRIANGLE_STRIP",t[t.TRIANGLE_FAN=6]="TRIANGLE_FAN"})(ce||(ce={}));function ps(t){switch(t){case ce.POINTS:return"point-list";case ce.LINES:return"line-list";case ce.LINE_STRIP:return"line-strip";case ce.TRIANGLES:return"triangle-list";case ce.TRIANGLE_STRIP:return"triangle-strip";default:throw new Error(String(t))}}var ms=F(oe(),1);var Bt=F(Fe(),1),ec=`
layout(0) positions: vec4; // in vec4 POSITION;

  #ifdef HAS_NORMALS
    in vec4 normals; // in vec4 NORMAL;
  #endif

  #ifdef HAS_TANGENTS
    in vec4 TANGENT;
  #endif

  #ifdef HAS_UV
    // in vec2 TEXCOORD_0;
    in vec2 texCoords;
  #endif

@vertex
  void main(void) {
    vec4 _NORMAL = vec4(0.);
    vec4 _TANGENT = vec4(0.);
    vec2 _TEXCOORD_0 = vec2(0.);

    #ifdef HAS_NORMALS
      _NORMAL = normals;
    #endif

    #ifdef HAS_TANGENTS
      _TANGENT = TANGENT;
    #endif

    #ifdef HAS_UV
      _TEXCOORD_0 = texCoords;
    #endif

    pbr_setPositionNormalTangentUV(positions, _NORMAL, _TANGENT, _TEXCOORD_0);
    gl_Position = u_MVPMatrix * positions;
  }

@fragment
  out vec4 fragmentColor;

  void main(void) {
    vec3 pos = pbr_vPosition;
    fragmentColor = pbr_filterColor(vec4(1.0));
  }
`,tc=`#version 300 es

  // in vec4 POSITION;
  in vec4 positions;

  #ifdef HAS_NORMALS
    // in vec4 NORMAL;
    in vec4 normals;
  #endif

  #ifdef HAS_TANGENTS
    in vec4 TANGENT;
  #endif

  #ifdef HAS_UV
    // in vec2 TEXCOORD_0;
    in vec2 texCoords;
  #endif

  void main(void) {
    vec4 _NORMAL = vec4(0.);
    vec4 _TANGENT = vec4(0.);
    vec2 _TEXCOORD_0 = vec2(0.);

    #ifdef HAS_NORMALS
      _NORMAL = normals;
    #endif

    #ifdef HAS_TANGENTS
      _TANGENT = TANGENT;
    #endif

    #ifdef HAS_UV
      _TEXCOORD_0 = texCoords;
    #endif

    pbr_setPositionNormalTangentUV(positions, _NORMAL, _TANGENT, _TEXCOORD_0);
    gl_Position = pbrProjection.modelViewProjectionMatrix * positions;
  }
`,nc=`#version 300 es
  out vec4 fragmentColor;

  void main(void) {
    vec3 pos = pbr_vPosition;
    fragmentColor = pbr_filterColor(vec4(1.0));
  }
`;function As(t,e){let{id:n,geometry:r,parsedPPBRMaterial:o,vertexCount:s,modelOptions:i={}}=e;ms.log.info(4,"createGLTFModel defines: ",o.defines)();let a=[],c={depthWriteEnabled:!0,depthCompare:"less",depthFormat:"depth24plus",cullMode:"back"},f={id:n,source:ec,vs:tc,fs:nc,geometry:r,topology:r.topology,vertexCount:s,modules:[Pe],...i,defines:{...o.defines,...i.defines},parameters:{...c,...o.parameters,...i.parameters}},l=new Bt.Model(t,f),{camera:h,...p}={...o.uniforms,...i.uniforms,...o.bindings,...i.bindings};return l.shaderInputs.setProps({pbrMaterial:p,pbrProjection:{camera:h}}),new Bt.ModelNode({managedResources:a,model:l})}var rc={modelOptions:{},pbrDebug:!1,imageBasedLightingEnvironment:void 0,lights:!0,useTangents:!1};function ds(t,e,n={}){let r={...rc,...n};return e.scenes.map(s=>oc(t,s,e.nodes,r))}function oc(t,e,n,r){let s=(e.nodes||[]).map(a=>gs(t,a,n,r));return new xe.GroupNode({id:e.name||e.id,children:s})}function gs(t,e,n,r){if(!e._node){let i=(e.children||[]).map(c=>gs(t,c,n,r));e.mesh&&i.push(sc(t,e.mesh,r));let a=new xe.GroupNode({id:e.name||e.id,children:i});if(e.matrix)a.setMatrix(e.matrix);else{if(a.matrix.identity(),e.translation&&a.matrix.translate(e.translation),e.rotation){let c=new q().fromQuaternion(e.rotation);a.matrix.multiplyRight(c)}e.scale&&a.matrix.scale(e.scale)}e._node=a}let o=n.find(s=>s.id===e.id);return o._node=e._node,e._node}function sc(t,e,n){if(!e._mesh){let o=(e.primitives||[]).map((i,a)=>ic(t,i,a,e,n)),s=new xe.GroupNode({id:e.name||e.id,children:o});e._mesh=s}return e._mesh}function ic(t,e,n,r,o){let s=e.name||`${r.name||r.id}-primitive-${n}`,i=ps(e.mode||4),a=e.indices?e.indices.count:ac(e.attributes),c=us(s,e,i),f=hs(t,e.material,c.attributes,o),l=As(t,{id:s,geometry:us(s,e,i),parsedPPBRMaterial:f,modelOptions:o.modelOptions,vertexCount:a});return l.bounds=[e.attributes.POSITION.min,e.attributes.POSITION.max],l}function ac(t){throw new Error("getVertexCount not implemented")}function us(t,e,n){let r={};for(let[o,s]of Object.entries(e.attributes)){let{components:i,size:a,value:c}=s;r[o]={size:a??i,value:c}}return new xe.Geometry({id:t,topology:n,indices:e.indices.value,attributes:r})}var xs=F(oe(),1);var En=F(oe(),1);var Cn=new Ge;function Bs(t,{input:e,interpolation:n,output:r},o,s){let i=e[e.length-1],a=t%i,c=e.findIndex(p=>p>=a),f=Math.max(0,c-1);if(!Array.isArray(o[s]))switch(s){case"translation":o[s]=[0,0,0];break;case"rotation":o[s]=[0,0,0,1];break;case"scale":o[s]=[1,1,1];break;default:En.log.warn(`Bad animation path ${s}`)()}let l=e[f],h=e[c];switch(n){case"STEP":lc(o,s,r[f]);break;case"LINEAR":if(h>l){let p=(a-l)/(h-l);cc(o,s,r[f],r[c],p)}break;case"CUBICSPLINE":if(h>l){let p=(a-l)/(h-l),m=h-l,A=r[3*f+1],u=r[3*f+2],d=r[3*c+0],B=r[3*c+1];fc(o,s,{p0:A,outTangent0:u,inTangent1:d,p1:B,tDiff:m,ratio:p})}break;default:En.log.warn(`Interpolation ${n} not supported`)();break}}function cc(t,e,n,r,o){if(!t[e])throw new Error;if(e==="rotation"){Cn.slerp({start:n,target:r,ratio:o});for(let s=0;s<Cn.length;s++)t[e][s]=Cn[s]}else for(let s=0;s<n.length;s++)t[e][s]=o*r[s]+(1-o)*n[s]}function fc(t,e,{p0:n,outTangent0:r,inTangent1:o,p1:s,tDiff:i,ratio:a}){if(!t[e])throw new Error;for(let c=0;c<t[e].length;c++){let f=r[c]*i,l=o[c]*i;t[e][c]=(2*Math.pow(a,3)-3*Math.pow(a,2)+1)*n[c]+(Math.pow(a,3)-2*Math.pow(a,2)+a)*f+(-2*Math.pow(a,3)+3*Math.pow(a,2))*s[c]+(Math.pow(a,3)-Math.pow(a,2))*l}}function lc(t,e,n){if(!t[e])throw new Error;for(let r=0;r<n.length;r++)t[e][r]=n[r]}var bn=class{animation;startTime=0;playing=!0;speed=1;constructor(e){this.animation=e.animation,this.animation.name||="unnamed",Object.assign(this,e)}setTime(e){if(!this.playing)return;let r=(e/1e3-this.startTime)*this.speed;this.animation.channels.forEach(({sampler:o,target:s,path:i})=>{Bs(r,o,s,i),pc(s,s._node)})}},xt=class{animations;constructor(e){this.animations=e.animations.map((n,r)=>{let o=n.name||`Animation-${r}`;return new bn({animation:{name:o,channels:n.channels}})})}animate(e){xs.log.warn("GLTFAnimator#animate is deprecated. Use GLTFAnimator#setTime instead")(),this.setTime(e)}setTime(e){this.animations.forEach(n=>n.setTime(e))}getAnimations(){return this.animations}},hc=new q;function pc(t,e){if(e.matrix.identity(),t.translation&&e.matrix.translate(t.translation),t.rotation){let n=hc.fromQuaternion(t.rotation);e.matrix.multiplyRight(n)}t.scale&&e.matrix.scale(t.scale)}var mc={SCALAR:1,VEC2:2,VEC3:3,VEC4:4,MAT2:4,MAT3:9,MAT4:16},Ac={5120:Int8Array,5121:Uint8Array,5122:Int16Array,5123:Uint16Array,5125:Uint32Array,5126:Float32Array};function Ms(t){let e=Ac[t.componentType],n=mc[t.type],r=n*t.count,{buffer:o,byteOffset:s=0}=t.bufferView?.data??{};return{typedArray:new e(o,s+(t.byteOffset||0),r),components:n}}function Es(t){return(t.animations||[]).map((n,r)=>{let o=n.name||`Animation-${r}`,s=n.samplers.map(({input:a,interpolation:c="LINEAR",output:f})=>({input:Cs(t.accessors[a]),interpolation:c,output:Cs(t.accessors[f])})),i=n.channels.map(({sampler:a,target:c})=>({sampler:s[a],target:t.nodes[c.node??0],path:c.path}));return{name:o,channels:i}})}function Cs(t){if(!t._animation){let{typedArray:e,components:n}=Ms(t);if(n===1)t._animation=Array.from(e);else{let r=[];for(let o=0;o<e.length;o+=n)r.push(Array.from(e.slice(o,o+n)));t._animation=r}}return t._animation}function Mt(t){if(ArrayBuffer.isView(t)||t instanceof ArrayBuffer||t instanceof ImageBitmap)return t;if(Array.isArray(t))return t.map(Mt);if(t&&typeof t=="object"){let e={};for(let n in t)e[n]=Mt(t[n]);return e}return t}function _n(t,e,n){e=Mt(e);let r=ds(t,e,n),o=Es(e),s=new xt({animations:o});return{scenes:r,animator:s}}var yn={};N(yn,{decode:()=>yc,name:()=>_c});function T(t,e){if(!t)throw new Error(e||"assert failed: gltf")}var Ct={SCALAR:1,VEC2:2,VEC3:3,VEC4:4,MAT2:4,MAT3:9,MAT4:16},Et={5120:1,5121:1,5122:2,5123:2,5125:4,5126:4};var bs=["SCALAR","VEC2","VEC3","VEC4"],uc=[[Int8Array,5120],[Uint8Array,5121],[Int16Array,5122],[Uint16Array,5123],[Uint32Array,5125],[Float32Array,5126],[Float64Array,5130]],dc=new Map(uc),gc={SCALAR:1,VEC2:2,VEC3:3,VEC4:4,MAT2:4,MAT3:9,MAT4:16},Bc={5120:1,5121:1,5122:2,5123:2,5125:4,5126:4},xc={5120:Int8Array,5121:Uint8Array,5122:Int16Array,5123:Uint16Array,5125:Uint32Array,5126:Float32Array};function bt(t){return bs[t-1]||bs[0]}function Me(t){let e=dc.get(t.constructor);if(!e)throw new Error("Illegal typed array");return e}function Ce(t,e){let n=xc[t.componentType],r=gc[t.type],o=Bc[t.componentType],s=t.count*r,i=t.count*r*o;T(i>=0&&i<=e.byteLength);let a=Et[t.componentType],c=Ct[t.type];return{ArrayType:n,length:s,byteLength:i,componentByteSize:a,numberOfComponentsInElement:c}}function _s(t,e,n){let r=t.bufferViews[n];T(r);let o=r.buffer,s=e[o];T(s);let i=(r.byteOffset||0)+s.byteOffset;return new Uint8Array(s.arrayBuffer,i,r.byteLength)}function ys(t,e,n){let r=typeof n=="number"?t.accessors?.[n]:n;if(!r)throw new Error(`No gltf accessor ${JSON.stringify(n)}`);let o=t.bufferViews?.[r.bufferView||0];if(!o)throw new Error(`No gltf buffer view for accessor ${o}`);let{arrayBuffer:s,byteOffset:i}=e[o.buffer],a=(i||0)+(r.byteOffset||0)+(o.byteOffset||0),{ArrayType:c,length:f,componentByteSize:l,numberOfComponentsInElement:h}=Ce(r,o),p=l*h,m=o.byteStride||p;if(typeof o.byteStride>"u"||o.byteStride===p)return new c(s,a,f);let A=new c(f);for(let u=0;u<r.count;u++){let d=new c(s,a+u*m,h);A.set(d,u*h)}return A}function Mc(){return{asset:{version:"2.0",generator:"loaders.gl"},buffers:[],extensions:{},extensionsRequired:[],extensionsUsed:[]}}var b=class{gltf;sourceBuffers;byteLength;constructor(e){this.gltf={json:e?.json||Mc(),buffers:e?.buffers||[],images:e?.images||[]},this.sourceBuffers=[],this.byteLength=0,this.gltf.buffers&&this.gltf.buffers[0]&&(this.byteLength=this.gltf.buffers[0].byteLength,this.sourceBuffers=[this.gltf.buffers[0]])}get json(){return this.gltf.json}getApplicationData(e){return this.json[e]}getExtraData(e){return(this.json.extras||{})[e]}hasExtension(e){let n=this.getUsedExtensions().find(o=>o===e),r=this.getRequiredExtensions().find(o=>o===e);return typeof n=="string"||typeof r=="string"}getExtension(e){let n=this.getUsedExtensions().find(o=>o===e),r=this.json.extensions||{};return n?r[e]:null}getRequiredExtension(e){return this.getRequiredExtensions().find(r=>r===e)?this.getExtension(e):null}getRequiredExtensions(){return this.json.extensionsRequired||[]}getUsedExtensions(){return this.json.extensionsUsed||[]}getRemovedExtensions(){return this.json.extensionsRemoved||[]}getObjectExtension(e,n){return(e.extensions||{})[n]}getScene(e){return this.getObject("scenes",e)}getNode(e){return this.getObject("nodes",e)}getSkin(e){return this.getObject("skins",e)}getMesh(e){return this.getObject("meshes",e)}getMaterial(e){return this.getObject("materials",e)}getAccessor(e){return this.getObject("accessors",e)}getTexture(e){return this.getObject("textures",e)}getSampler(e){return this.getObject("samplers",e)}getImage(e){return this.getObject("images",e)}getBufferView(e){return this.getObject("bufferViews",e)}getBuffer(e){return this.getObject("buffers",e)}getObject(e,n){if(typeof n=="object")return n;let r=this.json[e]&&this.json[e][n];if(!r)throw new Error(`glTF file error: Could not find ${e}[${n}]`);return r}getTypedArrayForBufferView(e){e=this.getBufferView(e);let n=e.buffer,r=this.gltf.buffers[n];T(r);let o=(e.byteOffset||0)+r.byteOffset;return new Uint8Array(r.arrayBuffer,o,e.byteLength)}getTypedArrayForAccessor(e){let n=this.getAccessor(e);return ys(this.gltf.json,this.gltf.buffers,n)}getTypedArrayForImageData(e){e=this.getAccessor(e);let n=this.getBufferView(e.bufferView),o=this.getBuffer(n.buffer).data,s=n.byteOffset||0;return new Uint8Array(o,s,n.byteLength)}addApplicationData(e,n){return this.json[e]=n,this}addExtraData(e,n){return this.json.extras=this.json.extras||{},this.json.extras[e]=n,this}addObjectExtension(e,n,r){return e.extensions=e.extensions||{},e.extensions[n]=r,this.registerUsedExtension(n),this}setObjectExtension(e,n,r){let o=e.extensions||{};o[n]=r}removeObjectExtension(e,n){let r=e?.extensions||{};if(r[n]){this.json.extensionsRemoved=this.json.extensionsRemoved||[];let o=this.json.extensionsRemoved;o.includes(n)||o.push(n)}delete r[n]}addExtension(e,n={}){return T(n),this.json.extensions=this.json.extensions||{},this.json.extensions[e]=n,this.registerUsedExtension(e),n}addRequiredExtension(e,n={}){return T(n),this.addExtension(e,n),this.registerRequiredExtension(e),n}registerUsedExtension(e){this.json.extensionsUsed=this.json.extensionsUsed||[],this.json.extensionsUsed.find(n=>n===e)||this.json.extensionsUsed.push(e)}registerRequiredExtension(e){this.registerUsedExtension(e),this.json.extensionsRequired=this.json.extensionsRequired||[],this.json.extensionsRequired.find(n=>n===e)||this.json.extensionsRequired.push(e)}removeExtension(e){if(this.json.extensions?.[e]){this.json.extensionsRemoved=this.json.extensionsRemoved||[];let n=this.json.extensionsRemoved;n.includes(e)||n.push(e)}this.json.extensions&&delete this.json.extensions[e],this.json.extensionsRequired&&this._removeStringFromArray(this.json.extensionsRequired,e),this.json.extensionsUsed&&this._removeStringFromArray(this.json.extensionsUsed,e)}setDefaultScene(e){this.json.scene=e}addScene(e){let{nodeIndices:n}=e;return this.json.scenes=this.json.scenes||[],this.json.scenes.push({nodes:n}),this.json.scenes.length-1}addNode(e){let{meshIndex:n,matrix:r}=e;this.json.nodes=this.json.nodes||[];let o={mesh:n};return r&&(o.matrix=r),this.json.nodes.push(o),this.json.nodes.length-1}addMesh(e){let{attributes:n,indices:r,material:o,mode:s=4}=e,a={primitives:[{attributes:this._addAttributes(n),mode:s}]};if(r){let c=this._addIndices(r);a.primitives[0].indices=c}return Number.isFinite(o)&&(a.primitives[0].material=o),this.json.meshes=this.json.meshes||[],this.json.meshes.push(a),this.json.meshes.length-1}addPointCloud(e){let r={primitives:[{attributes:this._addAttributes(e),mode:0}]};return this.json.meshes=this.json.meshes||[],this.json.meshes.push(r),this.json.meshes.length-1}addImage(e,n){let r=ae(e),o=n||r?.mimeType,i={bufferView:this.addBufferView(e),mimeType:o};return this.json.images=this.json.images||[],this.json.images.push(i),this.json.images.length-1}addBufferView(e,n=0,r=this.byteLength){let o=e.byteLength;T(Number.isFinite(o)),this.sourceBuffers=this.sourceBuffers||[],this.sourceBuffers.push(e);let s={buffer:n,byteOffset:r,byteLength:o};return this.byteLength+=ne(o,4),this.json.bufferViews=this.json.bufferViews||[],this.json.bufferViews.push(s),this.json.bufferViews.length-1}addAccessor(e,n){let r={bufferView:e,type:bt(n.size),componentType:n.componentType,count:n.count,max:n.max,min:n.min};return this.json.accessors=this.json.accessors||[],this.json.accessors.push(r),this.json.accessors.length-1}addBinaryBuffer(e,n={size:3}){let r=this.addBufferView(e),o={min:n.min,max:n.max};(!o.min||!o.max)&&(o=this._getAccessorMinMax(e,n.size));let s={size:n.size,componentType:Me(e),count:Math.round(e.length/n.size),min:o.min,max:o.max};return this.addAccessor(r,Object.assign(s,n))}addTexture(e){let{imageIndex:n}=e,r={source:n};return this.json.textures=this.json.textures||[],this.json.textures.push(r),this.json.textures.length-1}addMaterial(e){return this.json.materials=this.json.materials||[],this.json.materials.push(e),this.json.materials.length-1}createBinaryChunk(){this.gltf.buffers=[];let e=this.byteLength,n=new ArrayBuffer(e),r=new Uint8Array(n),o=0;for(let s of this.sourceBuffers||[])o=an(s,r,o);this.json?.buffers?.[0]?this.json.buffers[0].byteLength=e:this.json.buffers=[{byteLength:e}],this.gltf.binary=n,this.sourceBuffers=[n]}_removeStringFromArray(e,n){let r=!0;for(;r;){let o=e.indexOf(n);o>-1?e.splice(o,1):r=!1}}_addAttributes(e={}){let n={};for(let r in e){let o=e[r],s=this._getGltfAttributeName(r),i=this.addBinaryBuffer(o.value,o);n[s]=i}return n}_addIndices(e){return this.addBinaryBuffer(e,{size:1})}_getGltfAttributeName(e){switch(e.toLowerCase()){case"position":case"positions":case"vertices":return"POSITION";case"normal":case"normals":return"NORMAL";case"color":case"colors":return"COLOR_0";case"texcoord":case"texcoords":return"TEXCOORD_0";default:return e}}_getAccessorMinMax(e,n){let r={min:null,max:null};if(e.length<n)return r;r.min=[],r.max=[];let o=e.subarray(0,n);for(let s of o)r.min.push(s),r.max.push(s);for(let s=n;s<e.length;s+=n)for(let i=0;i<n;i++)r.min[0+i]=Math.min(r.min[0+i],e[s+i]),r.max[0+i]=Math.max(r.max[0+i],e[s+i]);return r}};function Ts(t){return(t%1+1)%1}var Rs={SCALAR:1,VEC2:2,VEC3:3,VEC4:4,MAT2:4,MAT3:9,MAT4:16,BOOLEAN:1,STRING:1,ENUM:1},Cc={INT8:Int8Array,UINT8:Uint8Array,INT16:Int16Array,UINT16:Uint16Array,INT32:Int32Array,UINT32:Uint32Array,INT64:BigInt64Array,UINT64:BigUint64Array,FLOAT32:Float32Array,FLOAT64:Float64Array},Is={INT8:1,UINT8:1,INT16:2,UINT16:2,INT32:4,UINT32:4,INT64:8,UINT64:8,FLOAT32:4,FLOAT64:8};function ke(t,e){return Is[e]*Rs[t]}function Ee(t,e,n,r){if(n!=="UINT8"&&n!=="UINT16"&&n!=="UINT32"&&n!=="UINT64")return null;let o=t.getTypedArrayForBufferView(e),s=be(o,"SCALAR",n,r+1);return s instanceof BigInt64Array||s instanceof BigUint64Array?null:s}function be(t,e,n,r=1){let o=Rs[e],s=Cc[n],i=Is[n],a=r*o,c=a*i,f=t.buffer,l=t.byteOffset;return l%i!==0&&(f=new Uint8Array(f).slice(l,l+c).buffer,l=0),new s(f,l,a)}function _e(t,e,n){let r=`TEXCOORD_${e.texCoord||0}`,o=n.attributes[r],s=t.getTypedArrayForAccessor(o),i=t.gltf.json,a=e.index,c=i.textures?.[a]?.source;if(typeof c<"u"){let f=i.images?.[c]?.mimeType,l=t.gltf.images?.[c];if(l&&typeof l.width<"u"){let h=[];for(let p=0;p<s.length;p+=2){let m=Ec(l,f,s,p,e.channels);h.push(m)}return h}}return[]}function _t(t,e,n,r,o){if(!n?.length)return;let s=[];for(let l of n){let h=r.findIndex(p=>p===l);h===-1&&(h=r.push(l)-1),s.push(h)}let i=new Uint32Array(s),a=t.gltf.buffers.push({arrayBuffer:i.buffer,byteOffset:i.byteOffset,byteLength:i.byteLength})-1,c=t.addBufferView(i,a,0),f=t.addAccessor(c,{size:1,componentType:Me(i),count:i.length});o.attributes[e]=f}function Ec(t,e,n,r,o=[0]){let s={r:{offset:0,shift:0},g:{offset:1,shift:8},b:{offset:2,shift:16},a:{offset:3,shift:24}},i=n[r],a=n[r+1],c=1;e&&(e.indexOf("image/jpeg")!==-1||e.indexOf("image/png")!==-1)&&(c=4);let f=bc(i,a,t,c),l=0;for(let h of o){let p=typeof h=="number"?Object.values(s)[h]:s[h],m=f+p.offset,A=Ke(t);if(A.data.length<=m)throw new Error(`${A.data.length} <= ${m}`);let u=A.data[m];l|=u<<p.shift}return l}function bc(t,e,n,r=1){let o=n.width,s=Ts(t)*(o-1),i=Math.round(s),a=n.height,c=Ts(e)*(a-1),f=Math.round(c),l=n.components?n.components:r;return(f*o+i)*l}function yt(t,e,n,r,o){let s=[];for(let i=0;i<e;i++){let a=n[i],c=n[i+1]-n[i];if(c+a>r)break;let f=a/o,l=c/o;s.push(t.slice(f,f+l))}return s}function Tt(t,e,n){let r=[];for(let o=0;o<e;o++){let s=o*n;r.push(t.slice(s,s+n))}return r}function Rt(t,e,n,r){if(n)throw new Error("Not implemented - arrayOffsets for strings is specified");if(r){let o=[],s=new TextDecoder("utf8"),i=0;for(let a=0;a<t;a++){let c=r[a+1]-r[a];if(c+i<=e.length){let f=e.subarray(i,c+i),l=s.decode(f);o.push(l),i+=c}}return o}return[]}var Ss="EXT_mesh_features",_c=Ss;async function yc(t,e){let n=new b(t);Tc(n,e)}function Tc(t,e){let n=t.gltf.json;if(n.meshes)for(let r of n.meshes)for(let o of r.primitives)Rc(t,o,e)}function Rc(t,e,n){if(!n?.gltf?.loadBuffers)return;let o=e.extensions?.[Ss]?.featureIds;if(o)for(let s of o){let i;if(typeof s.attribute<"u"){let a=`_FEATURE_ID_${s.attribute}`,c=e.attributes[a];i=t.getTypedArrayForAccessor(c)}else typeof s.texture<"u"&&n?.gltf?.loadImages?i=_e(t,s.texture,e):i=[];s.data=i}}var In={};N(In,{decode:()=>Sc,name:()=>Ic});var Tn="EXT_structural_metadata",Ic=Tn;async function Sc(t,e){let n=new b(t);Lc(n,e)}function Lc(t,e){if(!e.gltf?.loadBuffers)return;let n=t.getExtension(Tn);n&&(e.gltf?.loadImages&&Fc(t,n),Dc(t,n))}function Fc(t,e){let n=e.propertyTextures,r=t.gltf.json;if(n&&r.meshes)for(let o of r.meshes)for(let s of o.primitives)Gc(t,n,s,e)}function Dc(t,e){let n=e.schema;if(!n)return;let r=n.classes,o=e.propertyTables;if(r&&o)for(let s in r){let i=vc(o,s);i&&Pc(t,n,i)}}function vc(t,e){for(let n of t)if(n.class===e)return n;return null}function Gc(t,e,n,r){if(!e)return;let s=n.extensions?.[Tn]?.propertyTextures;if(s)for(let i of s){let a=e[i];Oc(t,a,n,r)}}function Oc(t,e,n,r){if(!e.properties)return;r.dataAttributeNames||(r.dataAttributeNames=[]);let o=e.class;for(let s in e.properties){let i=`${o}_${s}`,a=e.properties?.[s];if(!a)continue;a.data||(a.data=[]);let c=a.data,f=_e(t,a,n);f!==null&&(_t(t,i,f,c,n),a.data=c,r.dataAttributeNames.push(i))}}function Pc(t,e,n){let r=e.classes?.[n.class];if(!r)throw new Error(`Incorrect data in the EXT_structural_metadata extension: no schema class with name ${n.class}`);let o=n.count;for(let s in r.properties){let i=r.properties[s],a=n.properties?.[s];if(a){let c=wc(t,e,i,o,a);a.data=c}}}function wc(t,e,n,r,o){let s=[],i=o.values,a=t.getTypedArrayForBufferView(i),c=Nc(t,n,o,r),f=Uc(t,o,r);switch(n.type){case"SCALAR":case"VEC2":case"VEC3":case"VEC4":case"MAT2":case"MAT3":case"MAT4":{s=Hc(n,r,a,c);break}case"BOOLEAN":throw new Error(`Not implemented - classProperty.type=${n.type}`);case"STRING":{s=Rt(r,a,c,f);break}case"ENUM":{s=Jc(e,n,r,a,c);break}default:throw new Error(`Unknown classProperty type ${n.type}`)}return s}function Nc(t,e,n,r){return e.array&&typeof e.count>"u"&&typeof n.arrayOffsets<"u"?Ee(t,n.arrayOffsets,n.arrayOffsetType||"UINT32",r):null}function Uc(t,e,n){return typeof e.stringOffsets<"u"?Ee(t,e.stringOffsets,e.stringOffsetType||"UINT32",n):null}function Hc(t,e,n,r){let o=t.array,s=t.count,i=ke(t.type,t.componentType),a=n.byteLength/i,c;return t.componentType?c=be(n,t.type,t.componentType,a):c=n,o?r?yt(c,e,r,n.length,i):s?Tt(c,e,s):[]:c}function Jc(t,e,n,r,o){let s=e.enumType;if(!s)throw new Error("Incorrect data in the EXT_structural_metadata extension: classProperty.enumType is not set for type ENUM");let i=t.enums?.[s];if(!i)throw new Error(`Incorrect data in the EXT_structural_metadata extension: schema.enums does't contain ${s}`);let a=i.valueType||"UINT16",c=ke(e.type,a),f=r.byteLength/c,l=be(r,e.type,a,f);if(l||(l=r),e.array){if(o)return Kc({valuesData:l,numberOfElements:n,arrayOffsets:o,valuesDataBytesLength:r.length,elementSize:c,enumEntry:i});let h=e.count;return h?Vc(l,n,h,i):[]}return Rn(l,0,n,i)}function Kc(t){let{valuesData:e,numberOfElements:n,arrayOffsets:r,valuesDataBytesLength:o,elementSize:s,enumEntry:i}=t,a=[];for(let c=0;c<n;c++){let f=r[c],l=r[c+1]-r[c];if(l+f>o)break;let h=f/s,p=l/s,m=Rn(e,h,p,i);a.push(m)}return a}function Vc(t,e,n,r){let o=[];for(let s=0;s<e;s++){let i=n*s,a=Rn(t,i,n,r);o.push(a)}return o}function Rn(t,e,n,r){let o=[];for(let s=0;s<n;s++)if(t instanceof BigInt64Array||t instanceof BigUint64Array)o.push("");else{let i=t[e+s],a=Xc(r,i);a?o.push(a.name):o.push("")}return o}function Xc(t,e){for(let n of t.values)if(n.value===e)return n;return null}var Sn={};N(Sn,{decode:()=>kc,name:()=>jc});var Ls="EXT_feature_metadata",jc=Ls;async function kc(t,e){let n=new b(t);zc(n,e)}function zc(t,e){if(!e.gltf?.loadBuffers)return;let n=t.getExtension(Ls);n&&(e.gltf?.loadImages&&Yc(t,n),Qc(t,n))}function Yc(t,e){let n=e.schema;if(!n)return;let r=n.classes,{featureTextures:o}=e;if(r&&o)for(let s in r){let i=r[s],a=qc(o,s);a&&$c(t,a,i)}}function Qc(t,e){let n=e.schema;if(!n)return;let r=n.classes,o=e.featureTables;if(r&&o)for(let s in r){let i=Wc(o,s);i&&Zc(t,n,i)}}function Wc(t,e){for(let n in t){let r=t[n];if(r.class===e)return r}return null}function qc(t,e){for(let n in t){let r=t[n];if(r.class===e)return r}return null}function Zc(t,e,n){if(!n.class)return;let r=e.classes?.[n.class];if(!r)throw new Error(`Incorrect data in the EXT_structural_metadata extension: no schema class with name ${n.class}`);let o=n.count;for(let s in r.properties){let i=r.properties[s],a=n.properties?.[s];if(a){let c=ef(t,e,i,o,a);a.data=c}}}function $c(t,e,n){let r=e.class;for(let o in n.properties){let s=e?.properties?.[o];if(s){let i=sf(t,s,r);s.data=i}}}function ef(t,e,n,r,o){let s=[],i=o.bufferView,a=t.getTypedArrayForBufferView(i),c=tf(t,n,o,r),f=nf(t,n,o,r);return n.type==="STRING"||n.componentType==="STRING"?s=Rt(r,a,c,f):rf(n)&&(s=of(n,r,a,c)),s}function tf(t,e,n,r){return e.type==="ARRAY"&&typeof e.componentCount>"u"&&typeof n.arrayOffsetBufferView<"u"?Ee(t,n.arrayOffsetBufferView,n.offsetType||"UINT32",r):null}function nf(t,e,n,r){return typeof n.stringOffsetBufferView<"u"?Ee(t,n.stringOffsetBufferView,n.offsetType||"UINT32",r):null}function rf(t){let e=["UINT8","INT16","UINT16","INT32","UINT32","INT64","UINT64","FLOAT32","FLOAT64"];return e.includes(t.type)||typeof t.componentType<"u"&&e.includes(t.componentType)}function of(t,e,n,r){let o=t.type==="ARRAY",s=t.componentCount,i="SCALAR",a=t.componentType||t.type,c=ke(i,a),f=n.byteLength/c,l=be(n,i,a,f);return o?r?yt(l,e,r,n.length,c):s?Tt(l,e,s):[]:l}function sf(t,e,n){let r=t.gltf.json;if(!r.meshes)return[];let o=[];for(let s of r.meshes)for(let i of s.primitives)af(t,n,e,o,i);return o}function af(t,e,n,r,o){let s={channels:n.channels,...n.texture},i=_e(t,s,o);i&&_t(t,e,i,r,o)}var Fs="4.2.1";var ye=!0,Ds=1735152710,Dn=12,It=8,cf=1313821514,ff=5130562,lf=0,hf=0,pf=1;function mf(t,e=0){return`${String.fromCharCode(t.getUint8(e+0))}${String.fromCharCode(t.getUint8(e+1))}${String.fromCharCode(t.getUint8(e+2))}${String.fromCharCode(t.getUint8(e+3))}`}function vs(t,e=0,n={}){let r=new DataView(t),{magic:o=Ds}=n,s=r.getUint32(e,!1);return s===o||s===Ds}function Gs(t,e,n=0,r={}){let o=new DataView(e),s=mf(o,n+0),i=o.getUint32(n+4,ye),a=o.getUint32(n+8,ye);switch(Object.assign(t,{header:{byteOffset:n,byteLength:a,hasBinChunk:!1},type:s,version:i,json:{},binChunks:[]}),n+=Dn,t.version){case 1:return Af(t,o,n);case 2:return uf(t,o,n,r={});default:throw new Error(`Invalid GLB version ${t.version}. Only supports version 1 and 2.`)}}function Af(t,e,n){U(t.header.byteLength>Dn+It);let r=e.getUint32(n+0,ye),o=e.getUint32(n+4,ye);return n+=It,U(o===lf),Ln(t,e,n,r),n+=r,n+=Fn(t,e,n,t.header.byteLength),n}function uf(t,e,n,r){return U(t.header.byteLength>Dn+It),df(t,e,n,r),n+t.header.byteLength}function df(t,e,n,r){for(;n+8<=t.header.byteLength;){let o=e.getUint32(n+0,ye),s=e.getUint32(n+4,ye);switch(n+=It,s){case cf:Ln(t,e,n,o);break;case ff:Fn(t,e,n,o);break;case hf:r.strict||Ln(t,e,n,o);break;case pf:r.strict||Fn(t,e,n,o);break;default:break}n+=ne(o,4)}return n}function Ln(t,e,n,r){let o=new Uint8Array(e.buffer,n,r),i=new TextDecoder("utf8").decode(o);return t.json=JSON.parse(i),ne(r,4)}function Fn(t,e,n,r){return t.header.hasBinChunk=!0,t.binChunks.push({byteOffset:n,byteLength:r,arrayBuffer:e.buffer}),ne(r,4)}function vn(t,e){if(t.startsWith("data:")||t.startsWith("http:")||t.startsWith("https:"))return t;let r=e.baseUri||e.uri;if(!r)throw new Error(`'baseUri' must be provided to resolve relative url ${t}`);return r.substr(0,r.lastIndexOf("/")+1)+t}var On={};N(On,{decode:()=>If,name:()=>Rf});var gf="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",Bf="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",xf=new Uint8Array([0,97,115,109,1,0,0,0,1,4,1,96,0,0,3,3,2,0,0,5,3,1,0,1,12,1,0,10,22,2,12,0,65,0,65,0,65,0,252,10,0,0,11,7,0,65,0,253,15,26,11]),Mf=new Uint8Array([32,0,65,253,3,1,2,34,4,106,6,5,11,8,7,20,13,33,12,16,128,9,116,64,19,113,127,15,10,21,22,14,255,66,24,54,136,107,18,23,192,26,114,118,132,17,77,101,130,144,27,87,131,44,45,74,156,154,70,167]),Cf={0:"",1:"meshopt_decodeFilterOct",2:"meshopt_decodeFilterQuat",3:"meshopt_decodeFilterExp",NONE:"",OCTAHEDRAL:"meshopt_decodeFilterOct",QUATERNION:"meshopt_decodeFilterQuat",EXPONENTIAL:"meshopt_decodeFilterExp"},Ef={0:"meshopt_decodeVertexBuffer",1:"meshopt_decodeIndexBuffer",2:"meshopt_decodeIndexSequence",ATTRIBUTES:"meshopt_decodeVertexBuffer",TRIANGLES:"meshopt_decodeIndexBuffer",INDICES:"meshopt_decodeIndexSequence"};async function Os(t,e,n,r,o,s="NONE"){let i=await bf();Tf(i,i.exports[Ef[o]],t,e,n,r,i.exports[Cf[s||"NONE"]])}var Gn;async function bf(){return Gn||(Gn=_f()),Gn}async function _f(){let t=gf;WebAssembly.validate(xf)&&(t=Bf,console.log("Warning: meshopt_decoder is using experimental SIMD support"));let e=await WebAssembly.instantiate(yf(t),{});return await e.instance.exports.__wasm_call_ctors(),e.instance}function yf(t){let e=new Uint8Array(t.length);for(let r=0;r<t.length;++r){let o=t.charCodeAt(r);e[r]=o>96?o-71:o>64?o-65:o>47?o+4:o>46?63:62}let n=0;for(let r=0;r<t.length;++r)e[n++]=e[r]<60?Mf[e[r]]:(e[r]-60)*64+e[++r];return e.buffer.slice(0,n)}function Tf(t,e,n,r,o,s,i){let a=t.exports.sbrk,c=r+3&-4,f=a(c*o),l=a(s.length),h=new Uint8Array(t.exports.memory.buffer);h.set(s,l);let p=e(f,r,o,l,s.length);if(p===0&&i&&i(f,c,o),n.set(h.subarray(f,f+r*o)),a(f-a(0)),p!==0)throw new Error(`Malformed buffer data: ${p}`)}var St="EXT_meshopt_compression",Rf=St;async function If(t,e){let n=new b(t);if(!e?.gltf?.decompressMeshes||!e.gltf?.loadBuffers)return;let r=[];for(let o of t.json.bufferViews||[])r.push(Sf(n,o));await Promise.all(r),n.removeExtension(St)}async function Sf(t,e){let n=t.getObjectExtension(e,St);if(n){let{byteOffset:r=0,byteLength:o=0,byteStride:s,count:i,mode:a,filter:c="NONE",buffer:f}=n,l=t.gltf.buffers[f],h=new Uint8Array(l.arrayBuffer,l.byteOffset+r,o),p=new Uint8Array(t.gltf.buffers[e.buffer].arrayBuffer,e.byteOffset,e.byteLength);await Os(p,i,s,h,a,c),t.removeObjectExtension(e,St)}}var Pn={};N(Pn,{name:()=>Lf,preprocess:()=>Ff});var Te="EXT_texture_webp",Lf=Te;function Ff(t,e){let n=new b(t);if(!Mn("image/webp")){if(n.getRequiredExtensions().includes(Te))throw new Error(`gltf: Required extension ${Te} not supported by browser`);return}let{json:r}=n;for(let o of r.textures||[]){let s=n.getObjectExtension(o,Te);s&&(o.source=s.source),n.removeObjectExtension(o,Te)}n.removeExtension(Te)}var wn={};N(wn,{name:()=>Df,preprocess:()=>vf});var Lt="KHR_texture_basisu",Df=Lt;function vf(t,e){let n=new b(t),{json:r}=n;for(let o of r.textures||[]){let s=n.getObjectExtension(o,Lt);s&&(o.source=s.source,n.removeObjectExtension(o,Lt))}n.removeExtension(Lt)}var Kn={};N(Kn,{decode:()=>Qf,encode:()=>Wf,name:()=>zf,preprocess:()=>Yf});var Ps="4.2.1";var ws={dataType:null,batchType:null,name:"Draco",id:"draco",module:"draco",version:Ps,worker:!0,extensions:["drc"],mimeTypes:["application/octet-stream"],binary:!0,tests:["DRACO"],options:{draco:{decoderType:typeof WebAssembly=="object"?"wasm":"js",libraryPath:"libs/",extraAttributes:{},attributeNameEntry:void 0}}};function Us(t,e,n){let r=Hs(e.metadata),o=[],s=Gf(e.attributes);for(let i in t){let a=t[i],c=Ns(i,a,s[i]);o.push(c)}if(n){let i=Ns("indices",n);o.push(i)}return{fields:o,metadata:r}}function Gf(t){let e={};for(let n in t){let r=t[n];e[r.name||"undefined"]=r}return e}function Ns(t,e,n){let r=n?Hs(n.metadata):void 0;return en(t,e,r)}function Hs(t){Object.entries(t);let e={};for(let n in t)e[`${n}.string`]=JSON.stringify(t[n]);return e}var Js={POSITION:"POSITION",NORMAL:"NORMAL",COLOR:"COLOR_0",TEX_COORD:"TEXCOORD_0"},Of={1:Int8Array,2:Uint8Array,3:Int16Array,4:Uint16Array,5:Int32Array,6:Uint32Array,9:Float32Array},Pf=4,ze=class{draco;decoder;metadataQuerier;constructor(e){this.draco=e,this.decoder=new this.draco.Decoder,this.metadataQuerier=new this.draco.MetadataQuerier}destroy(){this.draco.destroy(this.decoder),this.draco.destroy(this.metadataQuerier)}parseSync(e,n={}){let r=new this.draco.DecoderBuffer;r.Init(new Int8Array(e),e.byteLength),this._disableAttributeTransforms(n);let o=this.decoder.GetEncodedGeometryType(r),s=o===this.draco.TRIANGULAR_MESH?new this.draco.Mesh:new this.draco.PointCloud;try{let i;switch(o){case this.draco.TRIANGULAR_MESH:i=this.decoder.DecodeBufferToMesh(r,s);break;case this.draco.POINT_CLOUD:i=this.decoder.DecodeBufferToPointCloud(r,s);break;default:throw new Error("DRACO: Unknown geometry type.")}if(!i.ok()||!s.ptr){let p=`DRACO decompression failed: ${i.error_msg()}`;throw new Error(p)}let a=this._getDracoLoaderData(s,o,n),c=this._getMeshData(s,a,n),f=we(c.attributes),l=Us(c.attributes,a,c.indices);return{loader:"draco",loaderData:a,header:{vertexCount:s.num_points(),boundingBox:f},...c,schema:l}}finally{this.draco.destroy(r),s&&this.draco.destroy(s)}}_getDracoLoaderData(e,n,r){let o=this._getTopLevelMetadata(e),s=this._getDracoAttributes(e,r);return{geometry_type:n,num_attributes:e.num_attributes(),num_points:e.num_points(),num_faces:e instanceof this.draco.Mesh?e.num_faces():0,metadata:o,attributes:s}}_getDracoAttributes(e,n){let r={};for(let o=0;o<e.num_attributes();o++){let s=this.decoder.GetAttribute(e,o),i=this._getAttributeMetadata(e,o);r[s.unique_id()]={unique_id:s.unique_id(),attribute_type:s.attribute_type(),data_type:s.data_type(),num_components:s.num_components(),byte_offset:s.byte_offset(),byte_stride:s.byte_stride(),normalized:s.normalized(),attribute_index:o,metadata:i};let a=this._getQuantizationTransform(s,n);a&&(r[s.unique_id()].quantization_transform=a);let c=this._getOctahedronTransform(s,n);c&&(r[s.unique_id()].octahedron_transform=c)}return r}_getMeshData(e,n,r){let o=this._getMeshAttributes(n,e,r);if(!o.POSITION)throw new Error("DRACO: No position attribute found.");if(e instanceof this.draco.Mesh)switch(r.topology){case"triangle-strip":return{topology:"triangle-strip",mode:4,attributes:o,indices:{value:this._getTriangleStripIndices(e),size:1}};case"triangle-list":default:return{topology:"triangle-list",mode:5,attributes:o,indices:{value:this._getTriangleListIndices(e),size:1}}}return{topology:"point-list",mode:0,attributes:o}}_getMeshAttributes(e,n,r){let o={};for(let s of Object.values(e.attributes)){let i=this._deduceAttributeName(s,r);s.name=i;let a=this._getAttributeValues(n,s);if(a){let{value:c,size:f}=a;o[i]={value:c,size:f,byteOffset:s.byte_offset,byteStride:s.byte_stride,normalized:s.normalized}}}return o}_getTriangleListIndices(e){let r=e.num_faces()*3,o=r*Pf,s=this.draco._malloc(o);try{return this.decoder.GetTrianglesUInt32Array(e,o,s),new Uint32Array(this.draco.HEAPF32.buffer,s,r).slice()}finally{this.draco._free(s)}}_getTriangleStripIndices(e){let n=new this.draco.DracoInt32Array;try{return this.decoder.GetTriangleStripsFromMesh(e,n),Uf(n)}finally{this.draco.destroy(n)}}_getAttributeValues(e,n){let r=Of[n.data_type];if(!r)return console.warn(`DRACO: Unsupported attribute type ${n.data_type}`),null;let o=n.num_components,i=e.num_points()*o,a=i*r.BYTES_PER_ELEMENT,c=wf(this.draco,r),f,l=this.draco._malloc(a);try{let h=this.decoder.GetAttribute(e,n.attribute_index);this.decoder.GetAttributeDataArrayForAllPoints(e,h,c,a,l),f=new r(this.draco.HEAPF32.buffer,l,i).slice()}finally{this.draco._free(l)}return{value:f,size:o}}_deduceAttributeName(e,n){let r=e.unique_id;for(let[i,a]of Object.entries(n.extraAttributes||{}))if(a===r)return i;let o=e.attribute_type;for(let i in Js)if(this.draco[i]===o)return Js[i];let s=n.attributeNameEntry||"name";return e.metadata[s]?e.metadata[s].string:`CUSTOM_ATTRIBUTE_${r}`}_getTopLevelMetadata(e){let n=this.decoder.GetMetadata(e);return this._getDracoMetadata(n)}_getAttributeMetadata(e,n){let r=this.decoder.GetAttributeMetadata(e,n);return this._getDracoMetadata(r)}_getDracoMetadata(e){if(!e||!e.ptr)return{};let n={},r=this.metadataQuerier.NumEntries(e);for(let o=0;o<r;o++){let s=this.metadataQuerier.GetEntryName(e,o);n[s]=this._getDracoMetadataField(e,s)}return n}_getDracoMetadataField(e,n){let r=new this.draco.DracoInt32Array;try{this.metadataQuerier.GetIntEntryArray(e,n,r);let o=Nf(r);return{int:this.metadataQuerier.GetIntEntry(e,n),string:this.metadataQuerier.GetStringEntry(e,n),double:this.metadataQuerier.GetDoubleEntry(e,n),intArray:o}}finally{this.draco.destroy(r)}}_disableAttributeTransforms(e){let{quantizedAttributes:n=[],octahedronAttributes:r=[]}=e,o=[...n,...r];for(let s of o)this.decoder.SkipAttributeTransform(this.draco[s])}_getQuantizationTransform(e,n){let{quantizedAttributes:r=[]}=n,o=e.attribute_type();if(r.map(i=>this.decoder[i]).includes(o)){let i=new this.draco.AttributeQuantizationTransform;try{if(i.InitFromAttribute(e))return{quantization_bits:i.quantization_bits(),range:i.range(),min_values:new Float32Array([1,2,3]).map(a=>i.min_value(a))}}finally{this.draco.destroy(i)}}return null}_getOctahedronTransform(e,n){let{octahedronAttributes:r=[]}=n,o=e.attribute_type();if(r.map(i=>this.decoder[i]).includes(o)){let i=new this.draco.AttributeQuantizationTransform;try{if(i.InitFromAttribute(e))return{quantization_bits:i.quantization_bits()}}finally{this.draco.destroy(i)}}return null}};function wf(t,e){switch(e){case Float32Array:return t.DT_FLOAT32;case Int8Array:return t.DT_INT8;case Int16Array:return t.DT_INT16;case Int32Array:return t.DT_INT32;case Uint8Array:return t.DT_UINT8;case Uint16Array:return t.DT_UINT16;case Uint32Array:return t.DT_UINT32;default:return t.DT_INVALID}}function Nf(t){let e=t.size(),n=new Int32Array(e);for(let r=0;r<e;r++)n[r]=t.GetValue(r);return n}function Uf(t){let e=t.size(),n=new Int32Array(e);for(let r=0;r<e;r++)n[r]=t.GetValue(r);return n}var Hf="1.5.6",Jf="1.4.1",Nn=`https://www.gstatic.com/draco/versioned/decoders/${Hf}`,D={DECODER:"draco_wasm_wrapper.js",DECODER_WASM:"draco_decoder.wasm",FALLBACK_DECODER:"draco_decoder.js",ENCODER:"draco_encoder.js"},Un={[D.DECODER]:`${Nn}/${D.DECODER}`,[D.DECODER_WASM]:`${Nn}/${D.DECODER_WASM}`,[D.FALLBACK_DECODER]:`${Nn}/${D.FALLBACK_DECODER}`,[D.ENCODER]:`https://raw.githubusercontent.com/google/draco/${Jf}/javascript/${D.ENCODER}`},Hn;async function Ks(t){let e=t.modules||{};return e.draco3d?Hn||=e.draco3d.createDecoderModule({}).then(n=>({draco:n})):Hn||=Kf(t),await Hn}async function Kf(t){let e,n;switch(t.draco&&t.draco.decoderType){case"js":e=await z(Un[D.FALLBACK_DECODER],"draco",t,D.FALLBACK_DECODER);break;case"wasm":default:[e,n]=await Promise.all([await z(Un[D.DECODER],"draco",t,D.DECODER),await z(Un[D.DECODER_WASM],"draco",t,D.DECODER_WASM)])}return e=e||globalThis.DracoDecoderModule,await Vf(e,n)}function Vf(t,e){let n={};return e&&(n.wasmBinary=e),new Promise(r=>{t({...n,onModuleLoaded:o=>r({draco:o})})})}var Vs={...ws,parse:Xf};async function Xf(t,e){let{draco:n}=await Ks(e),r=new ze(n);try{return r.parseSync(t,e?.draco)}finally{r.destroy()}}function Xs(t){let e={};for(let n in t){let r=t[n];if(n!=="indices"){let o=Jn(r);e[n]=o}}return e}function Jn(t){let{buffer:e,size:n,count:r}=jf(t);return{value:e,size:n,byteOffset:0,count:r,type:bt(n),componentType:Me(e)}}function jf(t){let e=t,n=1,r=0;return t&&t.value&&(e=t.value,n=t.size||1),e&&(ArrayBuffer.isView(e)||(e=kf(e,Float32Array)),r=e.length/n),{buffer:e,size:n,count:r}}function kf(t,e,n=!1){return t?Array.isArray(t)?new e(t):n&&!(t instanceof e)?new e(t):t:null}var re="KHR_draco_mesh_compression",zf=re;function Yf(t,e,n){let r=new b(t);for(let o of js(r))r.getObjectExtension(o,re)}async function Qf(t,e,n){if(!e?.gltf?.decompressMeshes)return;let r=new b(t),o=[];for(let s of js(r))r.getObjectExtension(s,re)&&o.push(qf(r,s,e,n));await Promise.all(o),r.removeExtension(re)}function Wf(t,e={}){let n=new b(t);for(let r of n.json.meshes||[])Zf(r,e),n.addRequiredExtension(re)}async function qf(t,e,n,r){let o=t.getObjectExtension(e,re);if(!o)return;let s=t.getTypedArrayForBufferView(o.bufferView),i=Je(s.buffer,s.byteOffset),a={...n};delete a["3d-tiles"];let c=await Ue(i,Vs,a,r),f=Xs(c.attributes);for(let[l,h]of Object.entries(f))if(l in e.attributes){let p=e.attributes[l],m=t.getAccessor(p);m?.min&&m?.max&&(h.min=m.min,h.max=m.max)}e.attributes=f,c.indices&&(e.indices=Jn(c.indices)),t.removeObjectExtension(e,re),$f(e)}function Zf(t,e,n=4,r,o){if(!r.DracoWriter)throw new Error("options.gltf.DracoWriter not provided");let s=r.DracoWriter.encodeSync({attributes:t}),i=o?.parseSync?.({attributes:t}),a=r._addFauxAttributes(i.attributes),c=r.addBufferView(s);return{primitives:[{attributes:a,mode:n,extensions:{[re]:{bufferView:c,attributes:a}}}]}}function $f(t){if(!t.attributes&&Object.keys(t.attributes).length>0)throw new Error("glTF: Empty primitive detected: Draco decompression failure?")}function*js(t){for(let e of t.json.meshes||[])for(let n of e.primitives)yield n}var Qn={};N(Qn,{decode:()=>ul,name:()=>pl});var ld=1/Math.PI*180,hd=1/180*Math.PI,el={EPSILON:1e-12,debug:!1,precision:4,printTypes:!1,printDegrees:!1,printRowMajor:!0,_cartographicRadians:!1};globalThis.mathgl=globalThis.mathgl||{config:{...el}};var I=globalThis.mathgl.config;function ks(t,{precision:e=I.precision}={}){return t=tl(t),`${parseFloat(t.toPrecision(e))}`}function Re(t){return Array.isArray(t)||ArrayBuffer.isView(t)&&!(t instanceof DataView)}function Vn(t,e,n){let r=I.EPSILON;n&&(I.EPSILON=n);try{if(t===e)return!0;if(Re(t)&&Re(e)){if(t.length!==e.length)return!1;for(let o=0;o<t.length;++o)if(!Vn(t[o],e[o]))return!1;return!0}return t&&t.equals?t.equals(e):e&&e.equals?e.equals(t):typeof t=="number"&&typeof e=="number"?Math.abs(t-e)<=I.EPSILON*Math.max(1,Math.abs(t),Math.abs(e)):!1}finally{I.EPSILON=r}}function tl(t){return Math.round(t/I.EPSILON)*I.EPSILON}var Ie=class extends Array{clone(){return new this.constructor().copy(this)}fromArray(e,n=0){for(let r=0;r<this.ELEMENTS;++r)this[r]=e[r+n];return this.check()}toArray(e=[],n=0){for(let r=0;r<this.ELEMENTS;++r)e[n+r]=this[r];return e}toObject(e){return e}from(e){return Array.isArray(e)?this.copy(e):this.fromObject(e)}to(e){return e===this?this:Re(e)?this.toArray(e):this.toObject(e)}toTarget(e){return e?this.to(e):this}toFloat32Array(){return new Float32Array(this)}toString(){return this.formatString(I)}formatString(e){let n="";for(let r=0;r<this.ELEMENTS;++r)n+=(r>0?", ":"")+ks(this[r],e);return`${e.printTypes?this.constructor.name:""}[${n}]`}equals(e){if(!e||this.length!==e.length)return!1;for(let n=0;n<this.ELEMENTS;++n)if(!Vn(this[n],e[n]))return!1;return!0}exactEquals(e){if(!e||this.length!==e.length)return!1;for(let n=0;n<this.ELEMENTS;++n)if(this[n]!==e[n])return!1;return!0}negate(){for(let e=0;e<this.ELEMENTS;++e)this[e]=-this[e];return this.check()}lerp(e,n,r){if(r===void 0)return this.lerp(this,e,n);for(let o=0;o<this.ELEMENTS;++o){let s=e[o],i=typeof n=="number"?n:n[o];this[o]=s+r*(i-s)}return this.check()}min(e){for(let n=0;n<this.ELEMENTS;++n)this[n]=Math.min(e[n],this[n]);return this.check()}max(e){for(let n=0;n<this.ELEMENTS;++n)this[n]=Math.max(e[n],this[n]);return this.check()}clamp(e,n){for(let r=0;r<this.ELEMENTS;++r)this[r]=Math.min(Math.max(this[r],e[r]),n[r]);return this.check()}add(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]+=n[r];return this.check()}subtract(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]-=n[r];return this.check()}scale(e){if(typeof e=="number")for(let n=0;n<this.ELEMENTS;++n)this[n]*=e;else for(let n=0;n<this.ELEMENTS&&n<e.length;++n)this[n]*=e[n];return this.check()}multiplyByScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]*=e;return this.check()}check(){if(I.debug&&!this.validate())throw new Error(`math.gl: ${this.constructor.name} some fields set to invalid numbers'`);return this}validate(){let e=this.length===this.ELEMENTS;for(let n=0;n<this.ELEMENTS;++n)e=e&&Number.isFinite(this[n]);return e}sub(e){return this.subtract(e)}setScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]=e;return this.check()}addScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]+=e;return this.check()}subScalar(e){return this.addScalar(-e)}multiplyScalar(e){for(let n=0;n<this.ELEMENTS;++n)this[n]*=e;return this.check()}divideScalar(e){return this.multiplyByScalar(1/e)}clampScalar(e,n){for(let r=0;r<this.ELEMENTS;++r)this[r]=Math.min(Math.max(this[r],e),n);return this.check()}get elements(){return this}};function nl(t,e){if(t.length!==e)return!1;for(let n=0;n<t.length;++n)if(!Number.isFinite(t[n]))return!1;return!0}function L(t){if(!Number.isFinite(t))throw new Error(`Invalid number ${JSON.stringify(t)}`);return t}function zs(t,e,n=""){if(I.debug&&!nl(t,e))throw new Error(`math.gl: ${n} some fields set to invalid numbers'`);return t}function Xn(t,e){if(!t)throw new Error(`math.gl assertion ${e}`)}var Ft=class extends Ie{get x(){return this[0]}set x(e){this[0]=L(e)}get y(){return this[1]}set y(e){this[1]=L(e)}len(){return Math.sqrt(this.lengthSquared())}magnitude(){return this.len()}lengthSquared(){let e=0;for(let n=0;n<this.ELEMENTS;++n)e+=this[n]*this[n];return e}magnitudeSquared(){return this.lengthSquared()}distance(e){return Math.sqrt(this.distanceSquared(e))}distanceSquared(e){let n=0;for(let r=0;r<this.ELEMENTS;++r){let o=this[r]-e[r];n+=o*o}return L(n)}dot(e){let n=0;for(let r=0;r<this.ELEMENTS;++r)n+=this[r]*e[r];return L(n)}normalize(){let e=this.magnitude();if(e!==0)for(let n=0;n<this.ELEMENTS;++n)this[n]/=e;return this.check()}multiply(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]*=n[r];return this.check()}divide(...e){for(let n of e)for(let r=0;r<this.ELEMENTS;++r)this[r]/=n[r];return this.check()}lengthSq(){return this.lengthSquared()}distanceTo(e){return this.distance(e)}distanceToSquared(e){return this.distanceSquared(e)}getComponent(e){return Xn(e>=0&&e<this.ELEMENTS,"index is out of range"),L(this[e])}setComponent(e,n){return Xn(e>=0&&e<this.ELEMENTS,"index is out of range"),this[e]=n,this.check()}addVectors(e,n){return this.copy(e).add(n)}subVectors(e,n){return this.copy(e).subtract(n)}multiplyVectors(e,n){return this.copy(e).multiply(n)}addScaledVector(e,n){return this.add(new this.constructor(e).multiplyScalar(n))}};var Se=typeof Float32Array<"u"?Float32Array:Array;var Ed=Math.PI/180;function il(){let t=new Se(2);return Se!=Float32Array&&(t[0]=0,t[1]=0),t}function Qs(t,e,n){let r=e[0],o=e[1];return t[0]=n[0]*r+n[3]*o+n[6],t[1]=n[1]*r+n[4]*o+n[7],t}var bd=function(){let t=il();return function(e,n,r,o,s,i){let a,c;for(n||(n=2),r||(r=0),o?c=Math.min(o*n+r,e.length):c=e.length,a=r;a<c;a+=n)t[0]=e[a],t[1]=e[a+1],s(t,t,i),e[a]=t[0],e[a+1]=t[1];return e}}();function Ws(t,e,n){let r=e[0],o=e[1],s=e[2],i=n[3]*r+n[7]*o+n[11]*s||1;return t[0]=(n[0]*r+n[4]*o+n[8]*s)/i,t[1]=(n[1]*r+n[5]*o+n[9]*s)/i,t[2]=(n[2]*r+n[6]*o+n[10]*s)/i,t}function qs(t,e,n){let r=e[0],o=e[1];return t[0]=n[0]*r+n[2]*o,t[1]=n[1]*r+n[3]*o,t[2]=e[2],t}function Zs(t,e,n){let r=e[0],o=e[1],s=e[2];return t[0]=n[0]*r+n[3]*o+n[6]*s,t[1]=n[1]*r+n[4]*o+n[7]*s,t[2]=n[2]*r+n[5]*o+n[8]*s,t[3]=e[3],t}function al(){let t=new Se(3);return Se!=Float32Array&&(t[0]=0,t[1]=0,t[2]=0),t}function cl(t,e){return t[0]*e[0]+t[1]*e[1]+t[2]*e[2]}function $s(t,e,n){let r=e[0],o=e[1],s=e[2],i=n[0],a=n[1],c=n[2];return t[0]=o*c-s*a,t[1]=s*i-r*c,t[2]=r*a-o*i,t}function ei(t,e,n){let r=e[0],o=e[1],s=e[2],i=n[3]*r+n[7]*o+n[11]*s+n[15];return i=i||1,t[0]=(n[0]*r+n[4]*o+n[8]*s+n[12])/i,t[1]=(n[1]*r+n[5]*o+n[9]*s+n[13])/i,t[2]=(n[2]*r+n[6]*o+n[10]*s+n[14])/i,t}function Dt(t,e,n){let r=e[0],o=e[1],s=e[2];return t[0]=r*n[0]+o*n[3]+s*n[6],t[1]=r*n[1]+o*n[4]+s*n[7],t[2]=r*n[2]+o*n[5]+s*n[8],t}function ti(t,e,n){let r=n[0],o=n[1],s=n[2],i=n[3],a=e[0],c=e[1],f=e[2],l=o*f-s*c,h=s*a-r*f,p=r*c-o*a,m=o*p-s*h,A=s*l-r*p,u=r*h-o*l,d=i*2;return l*=d,h*=d,p*=d,m*=2,A*=2,u*=2,t[0]=a+l+m,t[1]=c+h+A,t[2]=f+p+u,t}function ni(t,e,n,r){let o=[],s=[];return o[0]=e[0]-n[0],o[1]=e[1]-n[1],o[2]=e[2]-n[2],s[0]=o[0],s[1]=o[1]*Math.cos(r)-o[2]*Math.sin(r),s[2]=o[1]*Math.sin(r)+o[2]*Math.cos(r),t[0]=s[0]+n[0],t[1]=s[1]+n[1],t[2]=s[2]+n[2],t}function ri(t,e,n,r){let o=[],s=[];return o[0]=e[0]-n[0],o[1]=e[1]-n[1],o[2]=e[2]-n[2],s[0]=o[2]*Math.sin(r)+o[0]*Math.cos(r),s[1]=o[1],s[2]=o[2]*Math.cos(r)-o[0]*Math.sin(r),t[0]=s[0]+n[0],t[1]=s[1]+n[1],t[2]=s[2]+n[2],t}function oi(t,e,n,r){let o=[],s=[];return o[0]=e[0]-n[0],o[1]=e[1]-n[1],o[2]=e[2]-n[2],s[0]=o[0]*Math.cos(r)-o[1]*Math.sin(r),s[1]=o[0]*Math.sin(r)+o[1]*Math.cos(r),s[2]=o[2],t[0]=s[0]+n[0],t[1]=s[1]+n[1],t[2]=s[2]+n[2],t}function si(t,e){let n=t[0],r=t[1],o=t[2],s=e[0],i=e[1],a=e[2],c=Math.sqrt((n*n+r*r+o*o)*(s*s+i*i+a*a)),f=c&&cl(t,e)/c;return Math.acos(Math.min(Math.max(f,-1),1))}var Td=function(){let t=al();return function(e,n,r,o,s,i){let a,c;for(n||(n=3),r||(r=0),o?c=Math.min(o*n+r,e.length):c=e.length,a=r;a<c;a+=n)t[0]=e[a],t[1]=e[a+1],t[2]=e[a+2],s(t,t,i),e[a]=t[0],e[a+1]=t[1],e[a+2]=t[2];return e}}();var jn=[0,0,0],vt,fe=class extends Ft{static get ZERO(){return vt||(vt=new fe(0,0,0),Object.freeze(vt)),vt}constructor(e=0,n=0,r=0){super(-0,-0,-0),arguments.length===1&&Re(e)?this.copy(e):(I.debug&&(L(e),L(n),L(r)),this[0]=e,this[1]=n,this[2]=r)}set(e,n,r){return this[0]=e,this[1]=n,this[2]=r,this.check()}copy(e){return this[0]=e[0],this[1]=e[1],this[2]=e[2],this.check()}fromObject(e){return I.debug&&(L(e.x),L(e.y),L(e.z)),this[0]=e.x,this[1]=e.y,this[2]=e.z,this.check()}toObject(e){return e.x=this[0],e.y=this[1],e.z=this[2],e}get ELEMENTS(){return 3}get z(){return this[2]}set z(e){this[2]=L(e)}angle(e){return si(this,e)}cross(e){return $s(this,this,e),this.check()}rotateX({radians:e,origin:n=jn}){return ni(this,this,n,e),this.check()}rotateY({radians:e,origin:n=jn}){return ri(this,this,n,e),this.check()}rotateZ({radians:e,origin:n=jn}){return oi(this,this,n,e),this.check()}transform(e){return this.transformAsPoint(e)}transformAsPoint(e){return ei(this,this,e),this.check()}transformAsVector(e){return Ws(this,this,e),this.check()}transformByMatrix3(e){return Dt(this,this,e),this.check()}transformByMatrix2(e){return qs(this,this,e),this.check()}transformByQuaternion(e){return ti(this,this,e),this.check()}};var Gt=class extends Ie{toString(){let e="[";if(I.printRowMajor){e+="row-major:";for(let n=0;n<this.RANK;++n)for(let r=0;r<this.RANK;++r)e+=` ${this[r*this.RANK+n]}`}else{e+="column-major:";for(let n=0;n<this.ELEMENTS;++n)e+=` ${this[n]}`}return e+="]",e}getElementIndex(e,n){return n*this.RANK+e}getElement(e,n){return this[n*this.RANK+e]}setElement(e,n,r){return this[n*this.RANK+e]=L(r),this}getColumn(e,n=new Array(this.RANK).fill(-0)){let r=e*this.RANK;for(let o=0;o<this.RANK;++o)n[o]=this[r+o];return n}setColumn(e,n){let r=e*this.RANK;for(let o=0;o<this.RANK;++o)this[r+o]=n[o];return this}};function ii(t,e){if(t===e){let n=e[1],r=e[2],o=e[5];t[1]=e[3],t[2]=e[6],t[3]=n,t[5]=e[7],t[6]=r,t[7]=o}else t[0]=e[0],t[1]=e[3],t[2]=e[6],t[3]=e[1],t[4]=e[4],t[5]=e[7],t[6]=e[2],t[7]=e[5],t[8]=e[8];return t}function ai(t,e){let n=e[0],r=e[1],o=e[2],s=e[3],i=e[4],a=e[5],c=e[6],f=e[7],l=e[8],h=l*i-a*f,p=-l*s+a*c,m=f*s-i*c,A=n*h+r*p+o*m;return A?(A=1/A,t[0]=h*A,t[1]=(-l*r+o*f)*A,t[2]=(a*r-o*i)*A,t[3]=p*A,t[4]=(l*n-o*c)*A,t[5]=(-a*n+o*s)*A,t[6]=m*A,t[7]=(-f*n+r*c)*A,t[8]=(i*n-r*s)*A,t):null}function ci(t){let e=t[0],n=t[1],r=t[2],o=t[3],s=t[4],i=t[5],a=t[6],c=t[7],f=t[8];return e*(f*s-i*c)+n*(-f*o+i*a)+r*(c*o-s*a)}function kn(t,e,n){let r=e[0],o=e[1],s=e[2],i=e[3],a=e[4],c=e[5],f=e[6],l=e[7],h=e[8],p=n[0],m=n[1],A=n[2],u=n[3],d=n[4],B=n[5],E=n[6],g=n[7],x=n[8];return t[0]=p*r+m*i+A*f,t[1]=p*o+m*a+A*l,t[2]=p*s+m*c+A*h,t[3]=u*r+d*i+B*f,t[4]=u*o+d*a+B*l,t[5]=u*s+d*c+B*h,t[6]=E*r+g*i+x*f,t[7]=E*o+g*a+x*l,t[8]=E*s+g*c+x*h,t}function fi(t,e,n){let r=e[0],o=e[1],s=e[2],i=e[3],a=e[4],c=e[5],f=e[6],l=e[7],h=e[8],p=n[0],m=n[1];return t[0]=r,t[1]=o,t[2]=s,t[3]=i,t[4]=a,t[5]=c,t[6]=p*r+m*i+f,t[7]=p*o+m*a+l,t[8]=p*s+m*c+h,t}function li(t,e,n){let r=e[0],o=e[1],s=e[2],i=e[3],a=e[4],c=e[5],f=e[6],l=e[7],h=e[8],p=Math.sin(n),m=Math.cos(n);return t[0]=m*r+p*i,t[1]=m*o+p*a,t[2]=m*s+p*c,t[3]=m*i-p*r,t[4]=m*a-p*o,t[5]=m*c-p*s,t[6]=f,t[7]=l,t[8]=h,t}function zn(t,e,n){let r=n[0],o=n[1];return t[0]=r*e[0],t[1]=r*e[1],t[2]=r*e[2],t[3]=o*e[3],t[4]=o*e[4],t[5]=o*e[5],t[6]=e[6],t[7]=e[7],t[8]=e[8],t}function hi(t,e){let n=e[0],r=e[1],o=e[2],s=e[3],i=n+n,a=r+r,c=o+o,f=n*i,l=r*i,h=r*a,p=o*i,m=o*a,A=o*c,u=s*i,d=s*a,B=s*c;return t[0]=1-h-A,t[3]=l-B,t[6]=p+d,t[1]=l+B,t[4]=1-f-A,t[7]=m-u,t[2]=p-d,t[5]=m+u,t[8]=1-f-h,t}var Yn;(function(t){t[t.COL0ROW0=0]="COL0ROW0",t[t.COL0ROW1=1]="COL0ROW1",t[t.COL0ROW2=2]="COL0ROW2",t[t.COL1ROW0=3]="COL1ROW0",t[t.COL1ROW1=4]="COL1ROW1",t[t.COL1ROW2=5]="COL1ROW2",t[t.COL2ROW0=6]="COL2ROW0",t[t.COL2ROW1=7]="COL2ROW1",t[t.COL2ROW2=8]="COL2ROW2"})(Yn||(Yn={}));var fl=Object.freeze([1,0,0,0,1,0,0,0,1]),Z=class extends Gt{static get IDENTITY(){return hl()}static get ZERO(){return ll()}get ELEMENTS(){return 9}get RANK(){return 3}get INDICES(){return Yn}constructor(e,...n){super(-0,-0,-0,-0,-0,-0,-0,-0,-0),arguments.length===1&&Array.isArray(e)?this.copy(e):n.length>0?this.copy([e,...n]):this.identity()}copy(e){return this[0]=e[0],this[1]=e[1],this[2]=e[2],this[3]=e[3],this[4]=e[4],this[5]=e[5],this[6]=e[6],this[7]=e[7],this[8]=e[8],this.check()}identity(){return this.copy(fl)}fromObject(e){return this.check()}fromQuaternion(e){return hi(this,e),this.check()}set(e,n,r,o,s,i,a,c,f){return this[0]=e,this[1]=n,this[2]=r,this[3]=o,this[4]=s,this[5]=i,this[6]=a,this[7]=c,this[8]=f,this.check()}setRowMajor(e,n,r,o,s,i,a,c,f){return this[0]=e,this[1]=o,this[2]=a,this[3]=n,this[4]=s,this[5]=c,this[6]=r,this[7]=i,this[8]=f,this.check()}determinant(){return ci(this)}transpose(){return ii(this,this),this.check()}invert(){return ai(this,this),this.check()}multiplyLeft(e){return kn(this,e,this),this.check()}multiplyRight(e){return kn(this,this,e),this.check()}rotate(e){return li(this,this,e),this.check()}scale(e){return Array.isArray(e)?zn(this,this,e):zn(this,this,[e,e]),this.check()}translate(e){return fi(this,this,e),this.check()}transform(e,n){let r;switch(e.length){case 2:r=Qs(n||[-0,-0],e,this);break;case 3:r=Dt(n||[-0,-0,-0],e,this);break;case 4:r=Zs(n||[-0,-0,-0,-0],e,this);break;default:throw new Error("Illegal vector")}return zs(r,e.length),r}transformVector(e,n){return this.transform(e,n)}transformVector2(e,n){return this.transform(e,n)}transformVector3(e,n){return this.transform(e,n)}},Ot,Pt=null;function ll(){return Ot||(Ot=new Z([0,0,0,0,0,0,0,0,0]),Object.freeze(Ot)),Ot}function hl(){return Pt||(Pt=new Z,Object.freeze(Pt)),Pt}var Nt="KHR_texture_transform",pl=Nt,wt=new fe,ml=new Z,Al=new Z;async function ul(t,e){if(!new b(t).hasExtension(Nt)||!e.gltf?.loadBuffers)return;let o=t.json.materials||[];for(let s=0;s<o.length;s++)dl(s,t)}function dl(t,e){let n=e.json.materials?.[t],r=[n?.pbrMetallicRoughness?.baseColorTexture,n?.emissiveTexture,n?.normalTexture,n?.occlusionTexture,n?.pbrMetallicRoughness?.metallicRoughnessTexture],o=[];for(let s of r)s&&s?.extensions?.[Nt]&&gl(e,t,s,o)}function gl(t,e,n,r){let o=Bl(n,r);if(!o)return;let s=t.json.meshes||[];for(let i of s)for(let a of i.primitives){let c=a.material;Number.isFinite(c)&&e===c&&xl(t,a,o)}}function Bl(t,e){let n=t.extensions?.[Nt],{texCoord:r=0}=t,{texCoord:o=r}=n;if(!(e.findIndex(([i,a])=>i===r&&a===o)!==-1)){let i=El(n);return r!==o&&(t.texCoord=o),e.push([r,o]),{originalTexCoord:r,texCoord:o,matrix:i}}return null}function xl(t,e,n){let{originalTexCoord:r,texCoord:o,matrix:s}=n,i=e.attributes[`TEXCOORD_${r}`];if(Number.isFinite(i)){let a=t.json.accessors?.[i];if(a&&a.bufferView){let c=t.json.bufferViews?.[a.bufferView];if(c){let{arrayBuffer:f,byteOffset:l}=t.buffers[c.buffer],h=(l||0)+(a.byteOffset||0)+(c.byteOffset||0),{ArrayType:p,length:m}=Ce(a,c),A=Et[a.componentType],u=Ct[a.type],d=c.byteStride||A*u,B=new Float32Array(m);for(let E=0;E<a.count;E++){let g=new p(f,h+E*d,2);wt.set(g[0],g[1],1),wt.transformByMatrix3(s),B.set([wt[0],wt[1]],E*u)}r===o?Ml(a,c,t.buffers,B):Cl(o,a,e,t,B)}}}}function Ml(t,e,n,r){t.componentType=5126,n.push({arrayBuffer:r.buffer,byteOffset:0,byteLength:r.buffer.byteLength}),e.buffer=n.length-1,e.byteLength=r.buffer.byteLength,e.byteOffset=0,delete e.byteStride}function Cl(t,e,n,r,o){r.buffers.push({arrayBuffer:o.buffer,byteOffset:0,byteLength:o.buffer.byteLength});let s=r.json.bufferViews;if(!s)return;s.push({buffer:r.buffers.length-1,byteLength:o.buffer.byteLength,byteOffset:0});let i=r.json.accessors;i&&(i.push({bufferView:s?.length-1,byteOffset:0,componentType:5126,count:e.count,type:"VEC2"}),n.attributes[`TEXCOORD_${t}`]=i.length-1)}function El(t){let{offset:e=[0,0],rotation:n=0,scale:r=[1,1]}=t,o=new Z().set(1,0,0,0,1,0,e[0],e[1],1),s=ml.set(Math.cos(n),Math.sin(n),0,-Math.sin(n),Math.cos(n),0,0,0,1),i=Al.set(r[0],0,0,0,r[1],0,0,0,1);return o.multiplyRight(s).multiplyRight(i)}var Wn={};N(Wn,{decode:()=>_l,encode:()=>yl,name:()=>bl});var le="KHR_lights_punctual",bl=le;async function _l(t){let e=new b(t),{json:n}=e,r=e.getExtension(le);r&&(e.json.lights=r.lights,e.removeExtension(le));for(let o of n.nodes||[]){let s=e.getObjectExtension(o,le);s&&(o.light=s.light),e.removeObjectExtension(o,le)}}async function yl(t){let e=new b(t),{json:n}=e;if(n.lights){let r=e.addExtension(le);T(!r.lights),r.lights=n.lights,delete n.lights}if(e.json.lights){for(let r of e.json.lights){let o=r.node;e.addObjectExtension(o,le,r)}delete e.json.lights}}var qn={};N(qn,{decode:()=>Rl,encode:()=>Il,name:()=>Tl});var Ye="KHR_materials_unlit",Tl=Ye;async function Rl(t){let e=new b(t),{json:n}=e;for(let r of n.materials||[])r.extensions&&r.extensions.KHR_materials_unlit&&(r.unlit=!0),e.removeObjectExtension(r,Ye);e.removeExtension(Ye)}function Il(t){let e=new b(t),{json:n}=e;if(e.materials)for(let r of n.materials||[])r.unlit&&(delete r.unlit,e.addObjectExtension(r,Ye,{}),e.addExtension(Ye))}var Zn={};N(Zn,{decode:()=>Ll,encode:()=>Fl,name:()=>Sl});var Qe="KHR_techniques_webgl",Sl=Qe;async function Ll(t){let e=new b(t),{json:n}=e,r=e.getExtension(Qe);if(r){let o=Dl(r,e);for(let s of n.materials||[]){let i=e.getObjectExtension(s,Qe);i&&(s.technique=Object.assign({},i,o[i.technique]),s.technique.values=vl(s.technique,e)),e.removeObjectExtension(s,Qe)}e.removeExtension(Qe)}}async function Fl(t,e){}function Dl(t,e){let{programs:n=[],shaders:r=[],techniques:o=[]}=t,s=new TextDecoder;return r.forEach(i=>{if(Number.isFinite(i.bufferView))i.code=s.decode(e.getTypedArrayForBufferView(i.bufferView));else throw new Error("KHR_techniques_webgl: no shader code")}),n.forEach(i=>{i.fragmentShader=r[i.fragmentShader],i.vertexShader=r[i.vertexShader]}),o.forEach(i=>{i.program=n[i.program]}),o}function vl(t,e){let n=Object.assign({},t.values);return Object.keys(t.uniforms||{}).forEach(r=>{t.uniforms[r].value&&!(r in n)&&(n[r]=t.uniforms[r].value)}),Object.keys(n).forEach(r=>{typeof n[r]=="object"&&n[r].index!==void 0&&(n[r].texture=e.getTexture(n[r].index))}),n}var pi=[In,yn,On,Pn,wn,Kn,Wn,qn,Zn,Qn,Sn];function mi(t,e={},n){let r=pi.filter(o=>ui(o.name,e));for(let o of r)o.preprocess?.(t,e,n)}async function Ai(t,e={},n){let r=pi.filter(o=>ui(o.name,e));for(let o of r)await o.decode?.(t,e,n)}function ui(t,e){let n=e?.gltf?.excludeExtensions||{};return!(t in n&&!n[t])}var $n="KHR_binary_glTF";function di(t){let e=new b(t),{json:n}=e;for(let r of n.images||[]){let o=e.getObjectExtension(r,$n);o&&Object.assign(r,o),e.removeObjectExtension(r,$n)}n.buffers&&n.buffers[0]&&delete n.buffers[0].uri,e.removeExtension($n)}var gi={accessors:"accessor",animations:"animation",buffers:"buffer",bufferViews:"bufferView",images:"image",materials:"material",meshes:"mesh",nodes:"node",samplers:"sampler",scenes:"scene",skins:"skin",textures:"texture"},Ol={accessor:"accessors",animations:"animation",buffer:"buffers",bufferView:"bufferViews",image:"images",material:"materials",mesh:"meshes",node:"nodes",sampler:"samplers",scene:"scenes",skin:"skins",texture:"textures"},er=class{idToIndexMap={animations:{},accessors:{},buffers:{},bufferViews:{},images:{},materials:{},meshes:{},nodes:{},samplers:{},scenes:{},skins:{},textures:{}};json;normalize(e,n){this.json=e.json;let r=e.json;switch(r.asset&&r.asset.version){case"2.0":return;case void 0:case"1.0":break;default:console.warn(`glTF: Unknown version ${r.asset.version}`);return}if(!n.normalize)throw new Error("glTF v1 is not supported.");console.warn("Converting glTF v1 to glTF v2 format. This is experimental and may fail."),this._addAsset(r),this._convertTopLevelObjectsToArrays(r),di(e),this._convertObjectIdsToArrayIndices(r),this._updateObjects(r),this._updateMaterial(r)}_addAsset(e){e.asset=e.asset||{},e.asset.version="2.0",e.asset.generator=e.asset.generator||"Normalized to glTF 2.0 by loaders.gl"}_convertTopLevelObjectsToArrays(e){for(let n in gi)this._convertTopLevelObjectToArray(e,n)}_convertTopLevelObjectToArray(e,n){let r=e[n];if(!(!r||Array.isArray(r))){e[n]=[];for(let o in r){let s=r[o];s.id=s.id||o;let i=e[n].length;e[n].push(s),this.idToIndexMap[n][o]=i}}}_convertObjectIdsToArrayIndices(e){for(let n in gi)this._convertIdsToIndices(e,n);"scene"in e&&(e.scene=this._convertIdToIndex(e.scene,"scene"));for(let n of e.textures)this._convertTextureIds(n);for(let n of e.meshes)this._convertMeshIds(n);for(let n of e.nodes)this._convertNodeIds(n);for(let n of e.scenes)this._convertSceneIds(n)}_convertTextureIds(e){e.source&&(e.source=this._convertIdToIndex(e.source,"image"))}_convertMeshIds(e){for(let n of e.primitives){let{attributes:r,indices:o,material:s}=n;for(let i in r)r[i]=this._convertIdToIndex(r[i],"accessor");o&&(n.indices=this._convertIdToIndex(o,"accessor")),s&&(n.material=this._convertIdToIndex(s,"material"))}}_convertNodeIds(e){e.children&&(e.children=e.children.map(n=>this._convertIdToIndex(n,"node"))),e.meshes&&(e.meshes=e.meshes.map(n=>this._convertIdToIndex(n,"mesh")))}_convertSceneIds(e){e.nodes&&(e.nodes=e.nodes.map(n=>this._convertIdToIndex(n,"node")))}_convertIdsToIndices(e,n){e[n]||(console.warn(`gltf v1: json doesn't contain attribute ${n}`),e[n]=[]);for(let r of e[n])for(let o in r){let s=r[o],i=this._convertIdToIndex(s,o);r[o]=i}}_convertIdToIndex(e,n){let r=Ol[n];if(r in this.idToIndexMap){let o=this.idToIndexMap[r][e];if(!Number.isFinite(o))throw new Error(`gltf v1: failed to resolve ${n} with id ${e}`);return o}return e}_updateObjects(e){for(let n of this.json.buffers)delete n.type}_updateMaterial(e){for(let n of e.materials){n.pbrMetallicRoughness={baseColorFactor:[1,1,1,1],metallicFactor:1,roughnessFactor:1};let r=n.values?.tex||n.values?.texture2d_0||n.values?.diffuseTex,o=e.textures.findIndex(s=>s.id===r);o!==-1&&(n.pbrMetallicRoughness.baseColorTexture={index:o})}}};function Bi(t,e={}){return new er().normalize(t,e)}async function xi(t,e,n=0,r,o){return Pl(t,e,n,r),Bi(t,{normalize:r?.gltf?.normalize}),mi(t,r,o),r?.gltf?.loadBuffers&&t.json.buffers&&await wl(t,r,o),r?.gltf?.loadImages&&await Nl(t,r,o),await Ai(t,r,o),t}function Pl(t,e,n,r){if(r.uri&&(t.baseUri=r.uri),e instanceof ArrayBuffer&&!vs(e,n,r)&&(e=new TextDecoder().decode(e)),typeof e=="string")t.json=sn(e);else if(e instanceof ArrayBuffer){let i={};n=Gs(i,e,n,r.glb),T(i.type==="glTF",`Invalid GLB magic string ${i.type}`),t._glb=i,t.json=i.json}else T(!1,"GLTF: must be ArrayBuffer or string");let o=t.json.buffers||[];if(t.buffers=new Array(o.length).fill(null),t._glb&&t._glb.header.hasBinChunk){let{binChunks:i}=t._glb;t.buffers[0]={arrayBuffer:i[0].arrayBuffer,byteOffset:i[0].byteOffset,byteLength:i[0].byteLength}}let s=t.json.images||[];t.images=new Array(s.length).fill({})}async function wl(t,e,n){let r=t.json.buffers||[];for(let o=0;o<r.length;++o){let s=r[o];if(s.uri){let{fetch:i}=n;T(i);let a=vn(s.uri,e),f=await(await n?.fetch?.(a))?.arrayBuffer?.();t.buffers[o]={arrayBuffer:f,byteOffset:0,byteLength:f.byteLength},delete s.uri}else t.buffers[o]===null&&(t.buffers[o]={arrayBuffer:new ArrayBuffer(s.byteLength),byteOffset:0,byteLength:s.byteLength})}}async function Nl(t,e,n){let r=Ul(t),o=t.json.images||[],s=[];for(let i of r)s.push(Hl(t,o[i],i,e,n));return await Promise.all(s)}function Ul(t){let e=new Set,n=t.json.textures||[];for(let r of n)r.source!==void 0&&e.add(r.source);return Array.from(e).sort()}async function Hl(t,e,n,r,o){let s;if(e.uri&&!e.hasOwnProperty("bufferView")){let a=vn(e.uri,r),{fetch:c}=o;s=await(await c(a)).arrayBuffer(),e.bufferView={data:s}}if(Number.isFinite(e.bufferView)){let a=_s(t.json,t.buffers,e.bufferView);s=Je(a.buffer,a.byteOffset,a.byteLength)}T(s,"glTF image has no data");let i=await Ue(s,[Bn,mn],{...r,mimeType:e.mimeType,basis:r.basis||{format:ut()}},o);i&&i[0]&&(i={compressed:!0,mipmaps:!1,width:i[0].width,height:i[0].height,data:i[0]}),t.images=t.images||[],t.images[n]=i}var We={dataType:null,batchType:null,name:"glTF",id:"gltf",module:"gltf",version:Fs,extensions:["gltf","glb"],mimeTypes:["model/gltf+json","model/gltf-binary"],text:!0,binary:!0,tests:["glTF"],parse:Jl,options:{gltf:{normalize:!0,loadBuffers:!0,loadImages:!0,decompressMeshes:!0},log:console}};async function Jl(t,e={},n){e={...We.options,...e},e.gltf={...We.options.gltf,...e.gltf};let{byteOffset:r=0}=e;return await xi({},t,r,e,n)}var Kl={SCALAR:1,VEC2:2,VEC3:3,VEC4:4,MAT2:4,MAT3:9,MAT4:16},Vl={5120:1,5121:1,5122:2,5123:2,5125:4,5126:4},H={TEXTURE_MAG_FILTER:10240,TEXTURE_MIN_FILTER:10241,TEXTURE_WRAP_S:10242,TEXTURE_WRAP_T:10243,REPEAT:10497,LINEAR:9729,NEAREST_MIPMAP_LINEAR:9986},Xl={magFilter:H.TEXTURE_MAG_FILTER,minFilter:H.TEXTURE_MIN_FILTER,wrapS:H.TEXTURE_WRAP_S,wrapT:H.TEXTURE_WRAP_T},jl={[H.TEXTURE_MAG_FILTER]:H.LINEAR,[H.TEXTURE_MIN_FILTER]:H.NEAREST_MIPMAP_LINEAR,[H.TEXTURE_WRAP_S]:H.REPEAT,[H.TEXTURE_WRAP_T]:H.REPEAT};function kl(){return{id:"default-sampler",parameters:jl}}function zl(t){return Vl[t]}function Yl(t){return Kl[t]}var tr=class{baseUri="";jsonUnprocessed;json;buffers=[];images=[];postProcess(e,n={}){let{json:r,buffers:o=[],images:s=[]}=e,{baseUri:i=""}=e;return T(r),this.baseUri=i,this.buffers=o,this.images=s,this.jsonUnprocessed=r,this.json=this._resolveTree(e.json,n),this.json}_resolveTree(e,n={}){let r={...e};return this.json=r,e.bufferViews&&(r.bufferViews=e.bufferViews.map((o,s)=>this._resolveBufferView(o,s))),e.images&&(r.images=e.images.map((o,s)=>this._resolveImage(o,s))),e.samplers&&(r.samplers=e.samplers.map((o,s)=>this._resolveSampler(o,s))),e.textures&&(r.textures=e.textures.map((o,s)=>this._resolveTexture(o,s))),e.accessors&&(r.accessors=e.accessors.map((o,s)=>this._resolveAccessor(o,s))),e.materials&&(r.materials=e.materials.map((o,s)=>this._resolveMaterial(o,s))),e.meshes&&(r.meshes=e.meshes.map((o,s)=>this._resolveMesh(o,s))),e.nodes&&(r.nodes=e.nodes.map((o,s)=>this._resolveNode(o,s)),r.nodes=r.nodes.map((o,s)=>this._resolveNodeChildren(o))),e.skins&&(r.skins=e.skins.map((o,s)=>this._resolveSkin(o,s))),e.scenes&&(r.scenes=e.scenes.map((o,s)=>this._resolveScene(o,s))),typeof this.json.scene=="number"&&r.scenes&&(r.scene=r.scenes[this.json.scene]),r}getScene(e){return this._get(this.json.scenes,e)}getNode(e){return this._get(this.json.nodes,e)}getSkin(e){return this._get(this.json.skins,e)}getMesh(e){return this._get(this.json.meshes,e)}getMaterial(e){return this._get(this.json.materials,e)}getAccessor(e){return this._get(this.json.accessors,e)}getCamera(e){return this._get(this.json.cameras,e)}getTexture(e){return this._get(this.json.textures,e)}getSampler(e){return this._get(this.json.samplers,e)}getImage(e){return this._get(this.json.images,e)}getBufferView(e){return this._get(this.json.bufferViews,e)}getBuffer(e){return this._get(this.json.buffers,e)}_get(e,n){if(typeof n=="object")return n;let r=e&&e[n];return r||console.warn(`glTF file error: Could not find ${e}[${n}]`),r}_resolveScene(e,n){return{...e,id:e.id||`scene-${n}`,nodes:(e.nodes||[]).map(r=>this.getNode(r))}}_resolveNode(e,n){let r={...e,id:e?.id||`node-${n}`};return e.mesh!==void 0&&(r.mesh=this.getMesh(e.mesh)),e.camera!==void 0&&(r.camera=this.getCamera(e.camera)),e.skin!==void 0&&(r.skin=this.getSkin(e.skin)),e.meshes!==void 0&&e.meshes.length&&(r.mesh=e.meshes.reduce((o,s)=>{let i=this.getMesh(s);return o.id=i.id,o.primitives=o.primitives.concat(i.primitives),o},{primitives:[]})),r}_resolveNodeChildren(e){return e.children&&(e.children=e.children.map(n=>this.getNode(n))),e}_resolveSkin(e,n){let r=typeof e.inverseBindMatrices=="number"?this.getAccessor(e.inverseBindMatrices):void 0;return{...e,id:e.id||`skin-${n}`,inverseBindMatrices:r}}_resolveMesh(e,n){let r={...e,id:e.id||`mesh-${n}`,primitives:[]};return e.primitives&&(r.primitives=e.primitives.map(o=>{let s={...o,attributes:{},indices:void 0,material:void 0},i=o.attributes;for(let a in i)s.attributes[a]=this.getAccessor(i[a]);return o.indices!==void 0&&(s.indices=this.getAccessor(o.indices)),o.material!==void 0&&(s.material=this.getMaterial(o.material)),s})),r}_resolveMaterial(e,n){let r={...e,id:e.id||`material-${n}`};if(r.normalTexture&&(r.normalTexture={...r.normalTexture},r.normalTexture.texture=this.getTexture(r.normalTexture.index)),r.occlusionTexture&&(r.occlusionTexture={...r.occlusionTexture},r.occlusionTexture.texture=this.getTexture(r.occlusionTexture.index)),r.emissiveTexture&&(r.emissiveTexture={...r.emissiveTexture},r.emissiveTexture.texture=this.getTexture(r.emissiveTexture.index)),r.emissiveFactor||(r.emissiveFactor=r.emissiveTexture?[1,1,1]:[0,0,0]),r.pbrMetallicRoughness){r.pbrMetallicRoughness={...r.pbrMetallicRoughness};let o=r.pbrMetallicRoughness;o.baseColorTexture&&(o.baseColorTexture={...o.baseColorTexture},o.baseColorTexture.texture=this.getTexture(o.baseColorTexture.index)),o.metallicRoughnessTexture&&(o.metallicRoughnessTexture={...o.metallicRoughnessTexture},o.metallicRoughnessTexture.texture=this.getTexture(o.metallicRoughnessTexture.index))}return r}_resolveAccessor(e,n){let r=zl(e.componentType),o=Yl(e.type),s=r*o,i={...e,id:e.id||`accessor-${n}`,bytesPerComponent:r,components:o,bytesPerElement:s,value:void 0,bufferView:void 0,sparse:void 0};if(e.bufferView!==void 0&&(i.bufferView=this.getBufferView(e.bufferView)),i.bufferView){let a=i.bufferView.buffer,{ArrayType:c,byteLength:f}=Ce(i,i.bufferView),l=(i.bufferView.byteOffset||0)+(i.byteOffset||0)+a.byteOffset,h=a.arrayBuffer.slice(l,l+f);i.bufferView.byteStride&&(h=this._getValueFromInterleavedBuffer(a,l,i.bufferView.byteStride,i.bytesPerElement,i.count)),i.value=new c(h)}return i}_getValueFromInterleavedBuffer(e,n,r,o,s){let i=new Uint8Array(s*o);for(let a=0;a<s;a++){let c=n+a*r;i.set(new Uint8Array(e.arrayBuffer.slice(c,c+o)),a*o)}return i.buffer}_resolveTexture(e,n){return{...e,id:e.id||`texture-${n}`,sampler:typeof e.sampler=="number"?this.getSampler(e.sampler):kl(),source:typeof e.source=="number"?this.getImage(e.source):void 0}}_resolveSampler(e,n){let r={id:e.id||`sampler-${n}`,...e,parameters:{}};for(let o in r){let s=this._enumSamplerParameter(o);s!==void 0&&(r.parameters[s]=r[o])}return r}_enumSamplerParameter(e){return Xl[e]}_resolveImage(e,n){let r={...e,id:e.id||`image-${n}`,image:null,bufferView:e.bufferView!==void 0?this.getBufferView(e.bufferView):void 0},o=this.images[n];return o&&(r.image=o),r}_resolveBufferView(e,n){let r=e.buffer,o=this.buffers[r].arrayBuffer,s=this.buffers[r].byteOffset||0;return e.byteOffset&&(s+=e.byteOffset),{id:`bufferView-${n}`,...e,buffer:this.buffers[r],data:new Uint8Array(o,s,e.byteLength)}}_resolveCamera(e,n){let r={...e,id:e.id||`camera-${n}`};return r.perspective,r.orthographic,r}};function nr(t,e){return new tr().postProcess(t,e)}async function Mi(t){let e=[];return t.scenes.forEach(n=>{n.traverse(r=>{})}),await Ql(()=>e.some(n=>!n.loaded))}async function Ql(t){for(;t();)await new Promise(e=>requestAnimationFrame(e))}var Ci=`uniform scenegraphUniforms {
  float sizeScale;
  float sizeMinPixels;
  float sizeMaxPixels;
  mat4 sceneModelMatrix;
  bool composeModelMatrix;
} scenegraph;
`,Ei={name:"scenegraph",vs:Ci,fs:Ci,uniformTypes:{sizeScale:"f32",sizeMinPixels:"f32",sizeMaxPixels:"f32",sceneModelMatrix:"mat4x4<f32>",composeModelMatrix:"f32"}};var bi=`#version 300 es
#define SHADER_NAME scenegraph-layer-vertex-shader
in vec3 instancePositions;
in vec3 instancePositions64Low;
in vec4 instanceColors;
in vec3 instancePickingColors;
in vec3 instanceModelMatrixCol0;
in vec3 instanceModelMatrixCol1;
in vec3 instanceModelMatrixCol2;
in vec3 instanceTranslation;
in vec3 positions;
#ifdef HAS_UV
in vec2 texCoords;
#endif
#ifdef LIGHTING_PBR
#ifdef HAS_NORMALS
in vec3 normals;
#endif
#endif
out vec4 vColor;
#ifndef LIGHTING_PBR
#ifdef HAS_UV
out vec2 vTEXCOORD_0;
#endif
#endif
void main(void) {
#if defined(HAS_UV) && !defined(LIGHTING_PBR)
vTEXCOORD_0 = texCoords;
geometry.uv = texCoords;
#endif
geometry.worldPosition = instancePositions;
geometry.pickingColor = instancePickingColors;
mat3 instanceModelMatrix = mat3(instanceModelMatrixCol0, instanceModelMatrixCol1, instanceModelMatrixCol2);
vec3 normal = vec3(0.0, 0.0, 1.0);
#ifdef LIGHTING_PBR
#ifdef HAS_NORMALS
normal = instanceModelMatrix * (scenegraph.sceneModelMatrix * vec4(normals, 0.0)).xyz;
#endif
#endif
float originalSize = project_size_to_pixel(scenegraph.sizeScale);
float clampedSize = clamp(originalSize, scenegraph.sizeMinPixels, scenegraph.sizeMaxPixels);
vec3 pos = (instanceModelMatrix * (scenegraph.sceneModelMatrix * vec4(positions, 1.0)).xyz) * scenegraph.sizeScale * (clampedSize / originalSize) + instanceTranslation;
if(scenegraph.composeModelMatrix) {
DECKGL_FILTER_SIZE(pos, geometry);
geometry.normal = project_normal(normal);
geometry.worldPosition += pos;
gl_Position = project_position_to_clipspace(pos + instancePositions, instancePositions64Low, vec3(0.0), geometry.position);
}
else {
pos = project_size(pos);
DECKGL_FILTER_SIZE(pos, geometry);
gl_Position = project_position_to_clipspace(instancePositions, instancePositions64Low, pos, geometry.position);
geometry.normal = project_normal(normal);
}
DECKGL_FILTER_GL_POSITION(gl_Position, geometry);
#ifdef LIGHTING_PBR
pbr_vPosition = geometry.position.xyz;
#ifdef HAS_NORMALS
pbr_vNormal = geometry.normal;
#endif
#ifdef HAS_UV
pbr_vUV = texCoords;
#else
pbr_vUV = vec2(0., 0.);
#endif
geometry.uv = pbr_vUV;
#endif
vColor = instanceColors;
DECKGL_FILTER_COLOR(vColor, geometry);
}
`;var _i=`#version 300 es
#define SHADER_NAME scenegraph-layer-fragment-shader
in vec4 vColor;
out vec4 fragColor;
#ifndef LIGHTING_PBR
#if defined(HAS_UV) && defined(HAS_BASECOLORMAP)
in vec2 vTEXCOORD_0;
uniform sampler2D pbr_baseColorSampler;
#endif
#endif
void main(void) {
#ifdef LIGHTING_PBR
fragColor = vColor * pbr_filterColor(vec4(0));
geometry.uv = pbr_vUV;
#else
#if defined(HAS_UV) && defined(HAS_BASECOLORMAP)
fragColor = vColor * texture(pbr_baseColorSampler, vTEXCOORD_0);
geometry.uv = vTEXCOORD_0;
#else
fragColor = vColor;
#endif
#endif
fragColor.a *= layer.opacity;
DECKGL_FILTER_COLOR(fragColor, geometry);
}
`;var yi=[255,255,255,255],Wl={scenegraph:{type:"object",value:null,async:!0},getScene:t=>t&&t.scenes?typeof t.scene=="object"?t.scene:t.scenes[t.scene||0]:t,getAnimator:t=>t&&t.animator,_animations:null,sizeScale:{type:"number",value:1,min:0},sizeMinPixels:{type:"number",min:0,value:0},sizeMaxPixels:{type:"number",min:0,value:Number.MAX_SAFE_INTEGER},getPosition:{type:"accessor",value:t=>t.position},getColor:{type:"accessor",value:yi},_lighting:"flat",_imageBasedLightingEnvironment:void 0,getOrientation:{type:"accessor",value:[0,0,0]},getScale:{type:"accessor",value:[1,1,1]},getTranslation:{type:"accessor",value:[0,0,0]},getTransformMatrix:{type:"accessor",value:[]},loaders:[We]},qe=class extends Q.Layer{getShaders(){let e={},n;this.props._lighting==="pbr"?(n=Pe,e.LIGHTING_PBR=1):n={name:"pbrMaterial"};let r=[Q.project32,Q.picking,Ei,n];return super.getShaders({defines:e,vs:bi,fs:_i,modules:r})}initializeState(){this.getAttributeManager().addInstanced({instancePositions:{size:3,type:"float64",fp64:this.use64bitPositions(),accessor:"getPosition",transition:!0},instanceColors:{type:"unorm8",size:this.props.colorFormat.length,accessor:"getColor",defaultValue:yi,transition:!0},instanceModelMatrix:lt})}updateState(e){super.updateState(e);let{props:n,oldProps:r}=e;n.scenegraph!==r.scenegraph?this._updateScenegraph():n._animations!==r._animations&&this._applyAnimationsProp(this.state.animator,n._animations)}finalizeState(e){super.finalizeState(e),this.state.scenegraph?.destroy()}get isLoaded(){return Boolean(this.state?.scenegraph&&super.isLoaded)}_updateScenegraph(){let e=this.props,{device:n}=this.context,r=null;if(e.scenegraph instanceof he.ScenegraphNode)r={scenes:[e.scenegraph]};else if(e.scenegraph&&typeof e.scenegraph=="object"){let a=e.scenegraph,c=a.json?nr(a):a,f=_n(n,c,this._getModelOptions());r={gltf:c,...f},Mi(f).then(()=>{this.setNeedsRedraw()}).catch(l=>{this.raiseError(l,"loading glTF")})}let o={layer:this,device:this.context.device},s=e.getScene(r,o),i=e.getAnimator(r,o);if(s instanceof he.GroupNode){this.state.scenegraph?.destroy(),this._applyAnimationsProp(i,e._animations);let a=[];s.traverse(c=>{c instanceof he.ModelNode&&a.push(c.model)}),this.setState({scenegraph:s,animator:i,models:a}),this.getAttributeManager().invalidateAll()}else s!==null&&Q.log.warn("invalid scenegraph:",s)()}_applyAnimationsProp(e,n){if(!e||!n)return;let r=e.getAnimations();Object.keys(n).sort().forEach(o=>{let s=n[o];if(o==="*")r.forEach(i=>{Object.assign(i,s)});else if(Number.isFinite(Number(o))){let i=Number(o);i>=0&&i<r.length?Object.assign(r[i],s):Q.log.warn(`animation ${o} not found`)()}else{let i=r.find(({animation:a})=>a.name===o);i?Object.assign(i,s):Q.log.warn(`animation ${o} not found`)()}})}_getModelOptions(){let{_imageBasedLightingEnvironment:e}=this.props,n;return e&&(typeof e=="function"?n=e({gl:this.context.gl,layer:this}):n=e),{imageBasedLightingEnvironment:n,modelOptions:{id:this.props.id,isInstanced:!0,bufferLayout:this.getAttributeManager().getBufferLayouts(),...this.getShaders()},useTangents:!1}}draw({context:e}){if(!this.state.scenegraph)return;this.props._animations&&this.state.animator&&(this.state.animator.animate(e.timeline.getTime()),this.setNeedsRedraw());let{viewport:n,renderPass:r}=this.context,{sizeScale:o,sizeMinPixels:s,sizeMaxPixels:i,coordinateSystem:a}=this.props,c={camera:n.cameraPosition},f=this.getNumInstances();this.state.scenegraph.traverse((l,{worldMatrix:h})=>{if(l instanceof he.ModelNode){let{model:p}=l;p.setInstanceCount(f);let m={sizeScale:o,sizeMinPixels:s,sizeMaxPixels:i,composeModelMatrix:ht(n,a),sceneModelMatrix:h};p.shaderInputs.setProps({pbrProjection:c,scenegraph:m}),p.draw(r)}})}};qe.defaultProps=Wl;qe.layerName="ScenegraphLayer";var Ti=qe;return Di(Ze);})();
      return __exports__;
      });
