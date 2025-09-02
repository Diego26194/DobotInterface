// Components/Elements/Button1.tsx
import React from "react";
import Button, { ButtonProps } from "@mui/material/Button";

import { SxProps, Theme } from "@mui/material/styles";

type Button1Props = ButtonProps & {
  customSx?: SxProps<Theme>; // Estilos adicionales opcionales
};

const Button1: React.FC<Button1Props> = ({  sx, ...props }) => {
  return (
    <Button
      variant="contained"
      size="small"
      
      sx={{
        width: '75%',
        fontSize: "0.75rem",       // Tamaño de texto más compacto
        padding: "0px 12px",       // Padding reducido
        minHeight: "24px",         // Altura mínima
        textTransform: "none",   // Evita mayúsculas automáticas
        ...sx,                     // Permite sobrescribir desde afuera
      }}
        
      {...props}
    />
  );
};

export default Button1;
